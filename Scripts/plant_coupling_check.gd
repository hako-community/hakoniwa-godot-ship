extends Node
## 実物理（hakoniwa-mujoco-ship の ship_plant）との結合検証（--plant-test）。
##
## ship_plant が書く pos / vel / joint_states / radar_points と、外乱パブリッシャの maritime_disturbance を受信し、
##   1. 時刻: 物理は箱庭の許容遅延（conductor の max-delay）内でビューアより先に進む。joint_states.stamp − 箱庭時刻が
##      その範囲にあり、海面がその物理時刻で描かれていること（DisturbanceSync の physics_clock）
##   2. 海面と船体: 各船の上下位置と、ビューアが描いている海面（wave_spectrum.gd、船体水線面の平均）の相関。
##      船体の応答遅れ（0〜1.5 s）内で相関が高く、Tp/2 ずらした海面（時刻の取り違え）とは相関しないこと
##   3. レーダ: 新しい点群を受信したステップで、物標が他船の位置（RCS 一致）に重なり、波浪遮蔽の船は出ないこと。
##      物標は走査時刻の相対位置を現在の自船位置に置くので、走査からの経過 Δ の相対移動 (v_own − v_tgt)·Δ を補正する
##   4. 関節: レーダアンテナ角が 24 rpm × 物理時刻、舵角が針路保持の制限内、Godot のノードに反映
##   5. 運動: 姿勢列から推定した自船の対地速度が、物理の vel（船体固定系）と一致
## を検証して HAKONIWA_GODOT_SHIP_PLANT_OK で終了する。他船の速力・針路は参考表示（波浪中の操縦性能は物理側の課題）。
## 期待値は tests/integration/plant_coupling/expected.json。

const EXPECTED_PATH := "res://tests/integration/plant_coupling/expected.json"
const KT := 0.514444
const STATIONS := 10
const STEP_SEC := 0.02

var session: Node = null
var sim: Node = null
var fleet: Node3D = null
var ocean: Node = null
var overlays: Node = null
var radar_sync: Node = null
var dist_sync: Node = null

var expected: Dictionary = {}
var _own := ""
var _joint_endpoint = null
var _vel_endpoint = null
var _t := 0.0
var _stamp_offsets: Array = []      # joint_states.stamp − 箱庭時刻 [s]
var _clock_steps := 0
var _clock_from_physics := 0
var _lags: Array = []
var _heave: Dictionary = {}          # robot -> [相関の累積...]（lag ごと）
var _prev_joint_values := {}
var _joint_mappings: Array = []
var _vel_history: Array = []           # [t, v_phys(Vector2)]
var _joint_checks := 0
var _joint_mismatch := 0
var _radar_checked := 0
var _radar_clutter := 0
var _radar_max_err := 0.0
var _last_clouds := 0
var _vel_err_max := 0.0
var _vel_samples := 0
var _problems := PackedStringArray()
var _done := false

func _ready() -> void:
	expected = fleet._load_json(EXPECTED_PATH)
	if expected.is_empty():
		session._fail("HAKONIWA_GODOT_SHIP_PLANT_FAILED", "expected values not found: %s" % EXPECTED_PATH)
		return
	_own = str(expected["own_ship"])
	var tp := float(expected["wave"]["tp_s"])
	_lags = [0.0, 0.5, 1.0, 1.5, 0.5 * tp, -0.5 * tp]
	for robot in expected["ships"].keys():
		var list: Array = []
		for _l in _lags:
			list.append({"sx": 0.0, "sy": 0.0, "sxx": 0.0, "syy": 0.0, "sxy": 0.0, "sd2": 0.0, "n": 0})
		_heave[robot] = list
	# hakoniwa_robot_sync は call_deferred で simulation_step に接続する。こちらも遅らせて接続し、
	# 同じステップで関節・自船姿勢が反映された後に検証する
	call_deferred("_connect_signals")

func _connect_signals() -> void:
	sim.simulation_ready.connect(_on_simulation_ready)
	sim.simulation_step.connect(_on_simulation_step)

func _on_simulation_ready() -> void:
	# vel はこの検証だけが読む（lchannel を作る）。joint_states の lchannel はロボット同期が作る
	var ep = sim.get_endpoint()
	if ep != null and ep.create_pdu_lchannel(_own, "vel") == 0:
		_vel_endpoint = sim.get_typed_endpoint(_own, "vel")
	_joint_endpoint = sim.get_typed_endpoint(_own, "joint_states")
	_joint_mappings = fleet._load_json("res://config/robot_sync/%s.profile.json" % _own).get("joint_mappings", [])

func _on_simulation_step(simtime_usec: int, _world_time_usec: int) -> void:
	if _done:
		return
	_t = simtime_usec / 1.0e6
	var physics_t := _check_joint_states()
	_check_clock()
	_check_radar(physics_t)
	var settle := float(expected["settle_s"])
	if _t > settle:
		_sample_heave()
		_check_own_velocity()
	if _t > settle + float(expected["sample_s"]):
		_finish()

## joint_states を検証し、その stamp（物理時刻）を返す。未受信なら 0
func _check_joint_states() -> float:
	if _joint_endpoint == null:
		return 0.0
	var js: Dictionary = _joint_endpoint.recv_dict()
	var names: Array = js.get("name", [])
	if names.is_empty():
		return 0.0
	var stamp := _stamp_sec(js)
	_stamp_offsets.append(stamp - _t)
	var values := {}
	var positions: Array = js.get("position", [])
	for i in names.size():
		values[str(names[i])] = float(positions[i])
	var antenna := wrapf(TAU * float(expected["radar_rpm"]) / 60.0 * stamp, -PI, PI)
	if values.has("joint_radar_radar_xband") and absf(wrapf(values["joint_radar_radar_xband"] - antenna, -PI, PI)) > 1.0e-6:
		_add_problem("radar antenna angle differs from %.1f rpm x physics time" % float(expected["radar_rpm"]))
	var rudder_limit := deg_to_rad(float(expected["rudder_limit_deg"])) + 1.0e-3
	if absf(float(values.get("joint_rudder_main_rudder", 0.0))) > rudder_limit:
		_add_problem("rudder beyond heading-hold limit")
	# Godot のノードに反映されているか。ロボット同期はこのステップで、前回この検証が読んだ後・今回読む前に
	# 読んでいる（間に物理が書くこともある）→ ノードの角度が前回値と今回値の間（円弧上）にあればよい
	if not _prev_joint_values.is_empty():
		_joint_checks += 1
		var off := _joint_off_arc(_prev_joint_values, values)
		if not off.is_empty():
			_joint_mismatch += 1
			if _joint_mismatch == 1:
				_add_problem("joint node outside [previous, current] joint_states (first: %s)" % off)
	_prev_joint_values = values
	return stamp

## ロボット同期プロファイルの各関節について、ノードの回転角が prev〜cur の円弧上になければその関節名を返す
func _joint_off_arc(prev: Dictionary, cur: Dictionary) -> String:
	for mapping in _joint_mappings:
		var name := str(mapping["joint_name"])
		if not prev.has(name) or not cur.has(name):
			continue
		var b: Basis = (fleet.ships[_own].get_node(NodePath(str(mapping["node_path"]))) as Node3D).basis
		var phi := 0.0
		match str(mapping["axis"]):
			"x": phi = atan2(b.y.z, b.y.y)
			"y": phi = atan2(b.z.x, b.z.z)
			_: phi = atan2(b.x.y, b.x.x)
		var node_value := phi / float(mapping["sign"])
		var span := absf(wrapf(cur[name] - prev[name], -PI, PI))
		var d := absf(wrapf(node_value - prev[name], -PI, PI)) + absf(wrapf(cur[name] - node_value, -PI, PI))
		if d > span + 1.0e-4:
			return "%s node %.5f prev %.5f cur %.5f" % [name, node_value, prev[name], cur[name]]
	return ""

func _check_clock() -> void:
	if dist_sync == null:
		return
	_clock_steps += 1
	if dist_sync.physics_stamp_sec > 0.0 and is_equal_approx(ocean.sim_time_sec, dist_sync.physics_stamp_sec):
		_clock_from_physics += 1

## 新しい点群を受信したステップで、物標と他船の位置・RCS を照合する
func _check_radar(physics_t: float) -> void:
	if radar_sync == null or radar_sync.clouds_received == _last_clouds:
		return
	_last_clouds = radar_sync.clouds_received
	# 走査からの経過の補正に推定速度を使うので、速度推定の窓が埋まってから検証する
	if _t < float(expected["settle_s"]):
		return
	var delta := maxf(0.0, physics_t - radar_sync.last_stamp_sec) if physics_t > 0.0 else 0.0
	var tol := float(expected["radar_tolerance_m"])
	var own_v: Array = overlays.tracks[_own]["vel"]
	var used := {}
	for robot in expected["ships"].keys():
		if robot == _own:
			continue
		var cfg: Dictionary = expected["ships"][robot]
		var tr: Dictionary = overlays.tracks[robot]
		var shift := Vector2(own_v[0] - tr["vel"][0], own_v[1] - tr["vel"][1]) * delta
		var expect := Vector2(tr["pos"][0], tr["pos"][1]) + shift
		var best := INF
		var best_i := -1
		for i in overlays.radar_echoes_enu.size():
			var e: Array = overlays.radar_echoes_enu[i]
			var d := Vector2(e[0], e[1]).distance_to(expect)
			if d < best and absf(float(e[2]) - float(cfg["rcs_m2"])) <= 1.0e-3 * float(cfg["rcs_m2"]):
				best = d
				best_i = i
		var visible := bool(cfg["radar_visible"])
		if visible and best > tol:
			_add_problem("radar: %s not detected (nearest echo with its RCS %.2f m away, scan age %.2f s)" % [robot, best, delta])
		elif not visible and best <= tol:
			_add_problem("radar: %s must be hidden (%s) but echo at %.2f m" % [robot, cfg.get("_why", ""), best])
		if best_i >= 0 and best <= tol:
			used[best_i] = true
			_radar_max_err = maxf(_radar_max_err, best)
	# 物標以外はシークラッタ（RCS 小・クラッタ範囲内）
	var clutter_r: float = overlays.sea_clutter_radius_m()
	var own: Array = overlays.tracks[_own]["pos"]
	for i in overlays.radar_echoes_enu.size():
		if used.has(i):
			continue
		var e: Array = overlays.radar_echoes_enu[i]
		var r := Vector2(e[0] - own[0], e[1] - own[1]).length()
		if float(e[2]) > float(expected["clutter_rcs_max_m2"]) or r > clutter_r + 1.0:
			_add_problem("radar: unexplained echo rcs %.2f at %.1f m (clutter radius %.1f m)" % [e[2], r, clutter_r])
		else:
			_radar_clutter += 1
	_radar_checked += 1

## 姿勢列から推定した自船の対地速度（velocity_window_s の最小二乗）と、物理の vel（船体固定系 u, v を世界座標に）の
## 同じ窓での平均の差。瞬時値は波による前後揺れ（±0.2 m/s 程度）を含むので平均で比べる
func _check_own_velocity() -> void:
	if _vel_endpoint == null:
		return
	var vel: Dictionary = _vel_endpoint.recv_dict()
	var lin: Dictionary = vel.get("linear", {})
	if lin.is_empty():
		return
	var tr: Dictionary = overlays.tracks[_own]
	var h := deg_to_rad(float(tr["heading_deg"]))
	var fwd := Vector2(sin(h), cos(h))
	var left := Vector2(-cos(h), sin(h))
	var v_phys := fwd * float(lin.get("x", 0.0)) + left * float(lin.get("y", 0.0))
	var window := float(overlays.config.get("velocity_window_s", 5.0))
	_vel_history.append([_t, v_phys])
	while _vel_history.size() > 1 and _vel_history[0][0] < _t - window:
		_vel_history.pop_front()
	if _t < float(expected["settle_s"]) + window:
		return  # 窓が埋まってから比べる
	var mean := Vector2.ZERO
	for s in _vel_history:
		mean += s[1]
	mean /= _vel_history.size()
	_vel_err_max = maxf(_vel_err_max, mean.distance_to(Vector2(tr["vel"][0], tr["vel"][1])))
	_vel_samples += 1

func _sample_heave() -> void:
	var t_sea: float = ocean.sim_time_sec
	for robot in expected["ships"].keys():
		var cfg: Dictionary = expected["ships"][robot]
		var tr: Dictionary = overlays.tracks[robot]
		var z: float = fleet.ships[robot].global_position.y
		var h := deg_to_rad(float(tr["heading_deg"]))
		var fwd := Vector2(sin(h), cos(h))      # (東, 北)
		var left := Vector2(-cos(h), sin(h))
		var lpp := float(cfg["lpp_m"])
		var beam := float(cfg["beam_m"])
		for l in _lags.size():
			var tt: float = t_sea - float(_lags[l])
			var sum := 0.0
			for s in STATIONS:
				var xi := ((s + 0.5) / STATIONS - 0.5) * lpp
				for yi: float in [-0.5 * beam, 0.0, 0.5 * beam]:
					var p: Vector2 = Vector2(tr["pos"][0], tr["pos"][1]) + fwd * xi + left * yi
					sum += ocean.wave.get_elevation(p.x, p.y, tt)
			var eta := sum / (STATIONS * 3)
			var c: Dictionary = _heave[robot][l]
			c["sx"] += z; c["sy"] += eta; c["sxx"] += z * z; c["syy"] += eta * eta; c["sxy"] += z * eta
			c["sd2"] += (z - eta) * (z - eta); c["n"] += 1

func _finish() -> void:
	_done = true
	if _stamp_offsets.is_empty():
		_add_problem("no joint_states received")
	else:
		var lo: float = _stamp_offsets.min()
		var hi: float = _stamp_offsets.max()
		print("PLANT_TEST time: physics (joint_states.stamp) - viewer time in [%.4f, %.4f] s over %d steps" % [lo, hi, _stamp_offsets.size()])
		var max_ahead := float(expected["max_physics_lead_s"])
		if lo < -STEP_SEC - 1.0e-6 or hi > max_ahead + 1.0e-6:
			_add_problem("physics/viewer time offset [%.4f, %.4f] s outside [-%.2f, %.2f]" % [lo, hi, STEP_SEC, max_ahead])
	print("PLANT_TEST time: sea surface drawn at physics time in %d / %d steps" % [_clock_from_physics, _clock_steps])
	if _clock_from_physics < 0.99 * _clock_steps:
		_add_problem("sea surface not drawn at the physics time (%d / %d steps)" % [_clock_from_physics, _clock_steps])
	print("PLANT_TEST joints: Godot node angle outside [previous, current] joint_states in %d / %d steps" % [_joint_mismatch, _joint_checks])
	if _joint_checks == 0:
		_add_problem("joint nodes not checked")
	print("PLANT_TEST radar: %d scans checked, max target error %.3f m, clutter echoes %d" % [_radar_checked, _radar_max_err, _radar_clutter])
	if _radar_checked < 3:
		_add_problem("too few radar scans checked: %d" % _radar_checked)
	if _radar_clutter == 0:
		_add_problem("no sea clutter echoes")
	print("PLANT_TEST velocity: own ship estimate vs physics vel max error %.3f m/s over %d steps" % [_vel_err_max, _vel_samples])
	if _vel_samples == 0 or _vel_err_max > float(expected["velocity_tolerance_mps"]):
		_add_problem("own ship velocity estimate vs physics vel: max error %.3f m/s (%d samples)" % [_vel_err_max, _vel_samples])

	for robot in expected["ships"].keys():
		var cfg: Dictionary = expected["ships"][robot]
		var list: Array = _heave[robot]
		var line := "PLANT_TEST heave %-18s corr(z, eta(t-tau)):" % robot
		var best := -1.0
		var best_lag := 0.0
		for l in _lags.size():
			var r := _corr(list[l])
			line += " %+.1fs=%.3f" % [_lags[l], r]
			if l < 4 and r > best:
				best = r
				best_lag = _lags[l]
		var c0: Dictionary = list[0]
		var rms := sqrt(c0["sd2"] / maxf(1.0, c0["n"]))
		print("%s | best %.1f s, waterline rms %.3f m, n=%d" % [line, best_lag, rms, c0["n"]])
		if best < float(expected["heave_corr_min"]):
			_add_problem("%s: heave does not follow the viewer's sea surface (best corr %.3f)" % [robot, best])
		if _corr(list[4]) >= 0.3 or _corr(list[5]) >= 0.3:
			_add_problem("%s: heave correlates with the time-shifted sea surface" % robot)
		if rms > float(expected["waterline_rms_max_m"]):
			_add_problem("%s: waterline rms %.3f m" % [robot, rms])
		var tr: Dictionary = overlays.tracks[robot]
		var speed := Vector2(tr["vel"][0], tr["vel"][1]).length() / KT
		var scenario_heading := fposmod(90.0 - float(cfg["yaw_deg"]), 360.0)
		print("PLANT_TEST motion (info) %-18s speed %.2f kt (scenario %.1f), heading %.1f deg (scenario %.1f)" % [robot, speed, cfg["speed_kt"], tr["heading_deg"], scenario_heading])

	_problems.append_array(session._check_wave(expected["wave"]))
	var shot: String = session._arg_value("--screenshot=")
	if not shot.is_empty() and DisplayServer.get_name() != "headless":
		get_viewport().get_texture().get_image().save_png(shot)
		print("HAKONIWA_GODOT_SHIP_SCREENSHOT %s" % shot)
	if _problems.is_empty():
		print("HAKONIWA_GODOT_SHIP_PLANT_OK t=%.2f" % _t)
		get_tree().quit(0)
	else:
		session._fail("HAKONIWA_GODOT_SHIP_PLANT_FAILED", "\n  ".join(_problems))

static func _corr(c: Dictionary) -> float:
	var n := maxf(1.0, c["n"])
	var mx: float = c["sx"] / n
	var my: float = c["sy"] / n
	var vx: float = c["sxx"] / n - mx * mx
	var vy: float = c["syy"] / n - my * my
	return (c["sxy"] / n - mx * my) / sqrt(maxf(vx * vy, 1.0e-30))

static func _stamp_sec(msg: Dictionary) -> float:
	var stamp: Dictionary = msg.get("header", {}).get("stamp", {})
	return float(stamp.get("sec", 0)) + float(stamp.get("nanosec", 0)) * 1.0e-9

## 同じ問題を毎ステップ重複して記録しない
func _add_problem(p: String) -> void:
	if not _problems.has(p) and _problems.size() < 40:
		_problems.append(p)
