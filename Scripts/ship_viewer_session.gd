extends Node
## ビューアの起動モード管理と自己検証（SHP-23 Step 2〜4）。
##
## user args（`Godot ... -- <args>`）:
##   --smoke      オフラインで船隊生成・関節ノード解決・シーン向き・工学オーバレイを検証し HAKONIWA_GODOT_SHIP_BOOT_OK で終了
##                （オフライン時は preview_pose から preview_motion で等速直進させ、オーバレイを動かす）
##   --hako       HakoniwaSimNode を初期化して PDU 同期を有効化（共有メモリ Z:/mmap_ship、conductor が必要）
##   --hako-start --hako に加え、初期化後にシミュレーション開始を要求
##   --sync-test  --hako-start に加え、tests/integration/pdu_sync/expected.json の値（船の姿勢・関節・波浪・
##                等速直進・レーダ物標）を受信・反映でき、オーバレイ（速度推定・CPA/OZT・物標）が正しいか検証し終了
##   --plant-test --hako-start に加え、実物理（hakoniwa-mujoco-ship の ship_plant）との結合を検証し終了
##                （Scripts/plant_coupling_check.gd、期待値は tests/integration/plant_coupling/expected.json）
##   --screenshot=<path>  --smoke と併用（ウィンドウ実行時のみ）。検証後の画面を PNG 保存
##                        --plant-test と併用すると結合検証の終了時の画面を保存
##   --smoke-frames=<n>   --smoke の検証を行うフレーム（既定 10。航跡などを見たいときに増やす）
##   --no-overlays        工学オーバレイを表示しない（計算・検証は行う。船の外観の確認用）
##   --scenario=<path>    デモのシナリオ JSON。SUT の航路・岸壁を描く（SHP-31、Scripts/scenario_view.gd）
##   --run-dir=<path>     実行の結果の置き場。海域の航路（traffic/area_enu.json）を描く
##   --no-panel           センサ・通信の状態の画面を出さない
##   --shots=<t1,t2,...>  箱庭時刻 t [s] を過ぎたら画面を <run-dir>/godot_<t>.png に保存（ウィンドウ実行時のみ、目視確認用）

const SMOKE_FRAMES := 10
const SYNC_TEST_TIMEOUT_SEC := 30.0
const SYNC_EXPECTED_PATH := "res://tests/integration/pdu_sync/expected.json"
const POS_TOLERANCE_M := 1.0e-3
const ANGLE_TOLERANCE := 1.0e-3
## 速度推定（最小二乗）の許容誤差 [m/s]。オフラインは位置が厳密な等速直進、箱庭は受信時刻の揺らぎを含む
const VEL_TOLERANCE_OFFLINE := 0.02
const VEL_TOLERANCE_SYNC := 0.08
## 推定速度の誤差で OZT の境界の進路が入れ替わる分を許容する（2° 刻みの進路数）
const OZT_BOUNDARY_BINS := 2
const KT := 0.514444
const DisturbanceSync = preload("res://Scripts/disturbance_sync.gd")
const RadarEchoSync = preload("res://Scripts/radar_echo_sync.gd")
const CpaOzt = preload("res://Scripts/cpa_ozt.gd")
const PlantCouplingCheck = preload("res://Scripts/plant_coupling_check.gd")
const MonitorSync = preload("res://Scripts/monitor_sync.gd")
const TrafficView = preload("res://Scripts/traffic_view.gd")
const ScenarioView = preload("res://Scripts/scenario_view.gd")
const SensorPanel = preload("res://Scripts/sensor_panel.gd")
const PLANT_TEST_TIMEOUT_SEC := 240.0

@export var fleet_path: NodePath = ^"../Ships"
@export var sim_path: NodePath = ^"../Sim"
@export var camera_path: NodePath = ^"../Camera"
@export var sun_path: NodePath = ^"../Sun"
@export var ocean_path: NodePath = ^"../Ocean"
@export var overlays_path: NodePath = ^"../Overlays"
@export var hud_path: NodePath = ^"../Hud"
@export var environment_path: NodePath = ^"../WorldEnvironment"

var status_text: String = "offline"

var _args := PackedStringArray()
var _fleet: Node3D
var _sim: Node
var _ocean: Node
var _overlays: Node
var _offline_time := 0.0
var _frames := 0
var _sync_elapsed := 0.0
var _sync_expected: Dictionary = {}
var _sync_steps := 0
var _scenario_view: Node3D
var _shots: Array = []

func _ready() -> void:
	_args = OS.get_cmdline_user_args()
	_fleet = get_node(fleet_path)
	_sim = get_node(sim_path)
	_ocean = get_node(ocean_path)
	_overlays = get_node(overlays_path)

	var err: String = _fleet.build()
	if not err.is_empty():
		_fail("HAKONIWA_GODOT_SHIP_FLEET_FAILED", err)
		return
	_overlays.setup(_fleet, _ocean)
	if _has("--no-overlays"):
		_overlays.visible = false
	get_node(camera_path).target = _fleet.ships.get(_overlays.own_robot)
	_build_scenario_view()

	for v in _arg_value("--shots=").split(",", false):
		_shots.append(float(v))
	_shots.sort()
	if _has("--hako") or _has("--hako-start") or _has("--sync-test") or _has("--plant-test"):
		_start_hakoniwa()
	else:
		_fleet.apply_preview_poses()
		_ocean.set_wave_params(_fleet.fleet.get("preview_wave", {}))
		status_text = "offline (preview poses / preview_wave)"

func _physics_process(delta: float) -> void:
	_frames += 1
	if _sim.get_sync_state() == _sim.SYNC_STATE_UNINITIALIZED:
		# オフライン: ローカル時刻で波を進める（箱庭接続時は DisturbanceSync が sim 時刻を渡す）
		_offline_time += delta
		_ocean.set_time(_offline_time)
		_fleet.apply_preview_poses(_offline_time)
		_overlays.update(_offline_time)
	if not _shots.is_empty() and _sim.get_simtime_usec() / 1.0e6 >= float(_shots[0]):
		_save_shot(float(_shots.pop_front()))
	if _has("--smoke") and _frames == _smoke_frames():
		_run_smoke_checks()
	if _has("--plant-test"):
		_sync_elapsed += delta
		if _sync_elapsed > PLANT_TEST_TIMEOUT_SEC:
			_fail("HAKONIWA_GODOT_SHIP_PLANT_TIMEOUT", "no result within %.0f s (is ship_plant running?)" % PLANT_TEST_TIMEOUT_SEC)
	if _has("--sync-test") and not _sync_expected.is_empty():
		_sync_elapsed += delta
		var problems := _check_sync()
		if problems.is_empty():
			print("HAKONIWA_GODOT_SHIP_SYNC_OK steps=%d" % _sync_steps)
			get_tree().quit(0)
		elif _sync_elapsed > SYNC_TEST_TIMEOUT_SEC:
			_fail("HAKONIWA_GODOT_SHIP_SYNC_TIMEOUT", "\n  ".join(problems))

func _start_hakoniwa() -> void:
	_fleet.attach_sync(_sim)
	var disturbance: Dictionary = _fleet.fleet.get("disturbance", {})
	if not disturbance.is_empty():
		var dist_sync := DisturbanceSync.new()
		dist_sync.name = "DisturbanceSync"
		dist_sync.sim = _sim
		dist_sync.ocean = _ocean
		dist_sync.robot_name = str(disturbance["robot"])
		dist_sync.pdu_name = str(disturbance["pdu"])
		var clock: Dictionary = _fleet.fleet.get("physics_clock", {})
		dist_sync.clock_robot = str(clock.get("robot", ""))
		dist_sync.clock_pdu = str(clock.get("pdu", ""))
		add_child(dist_sync)
	var radar: Dictionary = _fleet.fleet.get("overlays", {}).get("radar", {})
	if radar.has("pdu"):
		var radar_sync := RadarEchoSync.new()
		radar_sync.name = "RadarEchoSync"
		radar_sync.sim = _sim
		radar_sync.overlays = _overlays
		radar_sync.robot_name = str(radar["robot"])
		radar_sync.pdu_name = str(radar["pdu"])
		add_child(radar_sync)
	_attach_monitor()
	_sim.simulation_step.connect(func(_s, _w): _sync_steps += 1)
	# 姿勢同期・レーダ受信の後に接続するので、同じステップで受信した値でオーバレイを更新する
	_sim.simulation_step.connect(func(simtime_usec, _w): _overlays.update(simtime_usec / 1.0e6))
	if _has("--plant-test"):
		# オーバレイ更新の後に接続する（同じステップの姿勢・物標で検証する）
		var check := PlantCouplingCheck.new()
		check.name = "PlantCouplingCheck"
		check.session = self
		check.sim = _sim
		check.fleet = _fleet
		check.ocean = _ocean
		check.overlays = _overlays
		check.radar_sync = get_node_or_null("RadarEchoSync")
		check.dist_sync = get_node_or_null("DisturbanceSync")
		add_child(check)
	var ret: int = _sim.initialize()
	if ret != 0:
		_fail("HAKONIWA_GODOT_SHIP_HAKO_INIT_FAILED", _sim.get_last_error_text())
		return
	status_text = "hakoniwa: initialized (%s)" % _sim.asset_name
	print("HAKONIWA_GODOT_SHIP_HAKO_INITIALIZED asset=%s" % _sim.asset_name)
	if _has("--hako-start") or _has("--sync-test") or _has("--plant-test"):
		ret = _sim.request_start()
		if ret != 0:
			_fail("HAKONIWA_GODOT_SHIP_HAKO_START_FAILED", _sim.get_last_error_text())
			return
		status_text = "hakoniwa: start requested"
	if _has("--sync-test"):
		_sync_expected = _fleet._load_json(SYNC_EXPECTED_PATH)
		if _sync_expected.is_empty():
			_fail("HAKONIWA_GODOT_SHIP_SYNC_FAILED", "expected values not found: %s" % SYNC_EXPECTED_PATH)

## シナリオに合わせた表示（航路・岸壁・海域の航路・GNSS・霧）。オフラインでも航路・岸壁は描く
func _build_scenario_view() -> void:
	var view := ScenarioView.new()
	_scenario_view = view
	view.name = "ScenarioView"
	view.fleet = _fleet
	view.own_robot = _overlays.own_robot
	var env_node := get_node_or_null(environment_path) as WorldEnvironment
	if env_node != null:
		view.environment = env_node.environment
	get_parent().add_child.call_deferred(view)
	view.setup(_arg_value("--scenario="), _arg_value("--run-dir="))

## センサ・通信の状態（SHP-31）：監視の PDU・交通の船・状態の画面
func _attach_monitor() -> void:
	var cfg: Dictionary = _fleet.fleet.get("monitor", {})
	if cfg.is_empty():
		return
	var monitor := MonitorSync.new()
	monitor.name = "MonitorSync"
	monitor.sim = _sim
	monitor.robot_name = str(cfg.get("robot", "OwnShip"))
	for key in cfg.keys():
		if key != "robot":
			monitor.pdus[key] = str(cfg[key])
	add_child(monitor)
	var dist_sync := get_node_or_null("DisturbanceSync")
	var traffic := TrafficView.new()
	traffic.name = "Traffic"
	traffic.monitor = monitor
	if dist_sync != null:
		traffic.clock = func() -> float: return dist_sync.physics_stamp_sec if dist_sync.physics_stamp_sec > 0.0 else monitor.sim_time
	get_parent().add_child.call_deferred(traffic)
	_scenario_view.monitor = monitor
	_scenario_view.dist_sync = dist_sync
	if not _has("--no-panel"):
		var panel := SensorPanel.new()
		panel.name = "SensorPanel"
		panel.monitor = monitor
		# 右上に固定（幅は内容に合わせて左へ伸びる）
		panel.anchor_left = 1.0
		panel.anchor_right = 1.0
		panel.offset_left = -610.0
		panel.offset_right = -10.0
		panel.offset_top = 40.0
		panel.grow_horizontal = Control.GROW_DIRECTION_BEGIN
		get_node(hud_path).add_child.call_deferred(panel)

func _save_shot(t: float) -> void:
	if DisplayServer.get_name() == "headless":
		return
	var dir := _arg_value("--run-dir=")
	var path := (dir if not dir.is_empty() else OS.get_user_data_dir()).path_join("godot_%04d.png" % int(t))
	get_viewport().get_texture().get_image().save_png(path)
	print("HAKONIWA_GODOT_SHIP_SCREENSHOT %s" % path)
	var monitor := get_node_or_null("MonitorSync")
	if monitor != null:
		# 画面と同じ時刻の受信値（検証用）
		var f := FileAccess.open(path.get_basename() + ".json", FileAccess.WRITE)
		if f != null:
			var traffic := get_parent().get_node_or_null("Traffic")
			f.store_string(JSON.stringify({"sim_time": monitor.sim_time, "plant_stamp": monitor.plant_stamp, "plant": monitor.plant_rows,
				"sut_stamp": monitor.sut_stamp, "sut": monitor.sut_rows, "autonomy": monitor.autonomy, "foc": monitor.foc,
				"gnss": monitor.gnss_fix, "traffic": monitor.traffic.size(), "traffic_models": traffic.model_info() if traffic != null else {}}, "  "))

func _run_smoke_checks() -> void:
	var problems := PackedStringArray()
	if _fleet.ships.size() != _fleet.get_ship_configs().size():
		problems.append("ships=%d expected=%d" % [_fleet.ships.size(), _fleet.get_ship_configs().size()])
	for missing in _fleet.find_missing_joint_nodes():
		problems.append("joint node not found: %s" % missing)
	# シーン向き: カメラ・太陽光とも下向き（.tscn の Transform3D は行優先表記のため取り違えやすい）
	for path in [camera_path, sun_path]:
		var forward := -(get_node(path) as Node3D).global_basis.z
		if forward.y >= 0.0:
			problems.append("%s faces upward: forward=%s" % [path, forward])
	# preview_wave が海面に展開されているか（物理プラグインと同じ 32 成分・有効化閾値）
	if not _ocean.wave_active or _ocean.wave.component_count() != _ocean.NUM_COMPONENTS:
		problems.append("ocean wave not active: active=%s components=%d" % [_ocean.wave_active, _ocean.wave.component_count()])
	elif _ocean.sim_time_sec <= 0.0:
		problems.append("ocean time not advancing")
	# preview_pose（＋ preview_motion の等速直進）が FLU→Godot 規約どおりに配置されているか
	var truth_vel := {}
	for ship in _fleet.get_ship_configs():
		var pose: Dictionary = _fleet.preview_pose_at(ship, _offline_time)
		problems.append_array(_check_pose(str(ship["robot"]), pose))
		var speed := float(ship.get("preview_motion", {}).get("sog_kt", 0.0)) * KT
		var yaw := float(pose.get("angular", {}).get("z", 0.0))
		truth_vel[str(ship["robot"])] = [speed * cos(yaw), speed * sin(yaw)]
	problems.append_array(_check_overlays(truth_vel, VEL_TOLERANCE_OFFLINE))
	if _overlays.ozt.values().all(func(s): return not s["has_danger"]):
		problems.append("preview scenario has no OZT danger (fleet.json preview_pose / preview_motion)")
	if _overlays.get_layer("radar").get_surface_count() == 0:
		problems.append("radar overlay not drawn")
	if not problems.is_empty():
		_fail("HAKONIWA_GODOT_SHIP_SMOKE_FAILED", "\n  ".join(problems))
		return
	var shot := _arg_value("--screenshot=")
	if not shot.is_empty() and DisplayServer.get_name() != "headless":
		get_viewport().get_texture().get_image().save_png(shot)
		print("HAKONIWA_GODOT_SHIP_SCREENSHOT %s" % shot)
	print("HAKONIWA_GODOT_SHIP_BOOT_OK ships=%d" % _fleet.ships.size())
	get_tree().quit(0)

func _check_sync() -> PackedStringArray:
	var problems := PackedStringArray()
	for robot in _sync_expected.keys():
		if robot.begins_with("_"):
			continue
		var expected: Dictionary = _sync_expected[robot]
		problems.append_array(_check_pose(robot, expected.get("pos", {}), _sync_expected.get("_motion", {}).get(robot, [])))
		if expected.has("joint_states"):
			problems.append_array(_check_joints(robot, expected["joint_states"]))
	if _sync_expected.has("_wave"):
		problems.append_array(_check_wave(_sync_expected["_wave"]))
	var motion: Dictionary = _sync_expected.get("_motion", {})
	if not motion.is_empty():
		var truth_vel := {}
		for robot in _fleet.ships.keys():
			truth_vel[robot] = motion.get(robot, [0.0, 0.0])
		# 速度推定は窓（velocity_window_s）がある程度埋まってから判定する
		var own_samples: Array = _overlays.tracks[_overlays.own_robot]["samples"]
		if own_samples.size() < 2 or own_samples.back()[0] - own_samples[0][0] < 2.0:
			problems.append("velocity window not filled yet")
		else:
			problems.append_array(_check_overlays(truth_vel, VEL_TOLERANCE_SYNC))
			if _overlays.ozt.values().all(func(s): return not s["has_danger"]):
				problems.append("sync scenario has no OZT danger")
	if _sync_expected.has("_radar"):
		problems.append_array(_check_radar(_sync_expected["_radar"]))
	return problems

## オーバレイの検証: 速度推定・航跡・タグ文言・CPA/OZT（真の速度で独立計算した値との比較）。
## truth_vel: robot -> ENU 速度 [m/s]
func _check_overlays(truth_vel: Dictionary, vel_tolerance: float) -> PackedStringArray:
	var problems := PackedStringArray()
	var own_robot: String = _overlays.own_robot
	for ship in _fleet.get_ship_configs():
		var robot := str(ship["robot"])
		var tr: Dictionary = _overlays.tracks[robot]
		var v: Array = truth_vel[robot]
		var err := Vector2(tr["vel"][0] - v[0], tr["vel"][1] - v[1]).length()
		if err > vel_tolerance:
			problems.append("%s: estimated velocity %s expected %s (err %.4f m/s)" % [robot, tr["vel"], v, err])
		if tr["trail"].is_empty():
			problems.append("%s: no trail points" % robot)
		var text: String = _overlays.get_label_text(robot)
		var ais: Dictionary = ship.get("ais", {})
		if robot == own_robot:
			if not text.contains("(OWN)"):
				problems.append("%s: own ship tag %s" % [robot, text.c_escape()])
		elif bool(ais.get("has_ais", false)):
			if not text.contains("MMSI %d\n" % int(ais.get("mmsi", 0))) or not text.contains(str(ais.get("name", ""))):
				problems.append("%s: AIS tag %s" % [robot, text.c_escape()])
		elif not text.contains("NON-AIS"):
			problems.append("%s: non-AIS tag %s" % [robot, text.c_escape()])
	if not problems.is_empty():
		return problems

	# CPA/OZT: 表示中の自船・他船位置と真の速度から独立に計算した結果と比較
	var cfg: Dictionary = _overlays.config.get("ozt", {})
	var safe_m := float(cfg.get("safe_domain_m", 926.0))
	var horizon_s := float(cfg.get("time_horizon_s", 900.0))
	var own_pos: Array = _overlays.tracks[own_robot]["pos"]
	var own_v: Array = truth_vel[own_robot]
	var own_speed := Vector2(own_v[0], own_v[1]).length()
	for robot in _fleet.ships.keys():
		if robot == own_robot:
			continue
		if not _overlays.ozt.has(robot):
			problems.append("%s: OZT not computed" % robot)
			continue
		var tgt_pos: Array = _overlays.tracks[robot]["pos"]
		var expected := CpaOzt.compute_ozt(own_pos, own_speed, tgt_pos, truth_vel[robot], safe_m, horizon_s)
		var actual: Dictionary = _overlays.ozt[robot]
		var diff := 0
		for c in actual["danger_courses_deg"]:
			if not expected["danger_courses_deg"].has(c):
				diff += 1
		for c in expected["danger_courses_deg"]:
			if not actual["danger_courses_deg"].has(c):
				diff += 1
		if diff > OZT_BOUNDARY_BINS:
			problems.append("%s: OZT courses %s expected %s" % [robot, CpaOzt.danger_arcs(actual["danger_courses_deg"]), CpaOzt.danger_arcs(expected["danger_courses_deg"])])
		var m := CpaOzt.compute_cpa(own_pos, own_v, atan2(own_v[0], own_v[1]), tgt_pos, truth_vel[robot], safe_m, horizon_s)
		if absf(float(_overlays.cpa[robot]["cpa_distance_m"]) - float(m["cpa_distance_m"])) > 10.0:
			problems.append("%s: CPA %.1f m expected %.1f m" % [robot, _overlays.cpa[robot]["cpa_distance_m"], m["cpa_distance_m"]])
	return problems

## レーダ物標: 受信点数と、自船の船首・左舷方向（Godot の基底から独立に求める）で置いた世界位置の一致。
## 原点はオーバレイが物標を置いたときの自船位置（検証時点では船が 1 ステップ進んでいるため）。
func _check_radar(points: Array) -> PackedStringArray:
	var problems := PackedStringArray()
	var echoes: Array = _overlays.radar_echoes_enu
	if echoes.size() != points.size():
		return PackedStringArray(["radar echoes %d expected %d" % [echoes.size(), points.size()]])
	var own: Node3D = _fleet.ships[_overlays.own_robot]
	var model: Node3D = own.get_node("Adapter/Model")
	var fwd := Vector3(-model.global_basis.z.x, 0.0, -model.global_basis.z.z).normalized()
	var left := Vector3.UP.cross(fwd)
	for i in points.size():
		var p: Array = points[i]
		var o: Array = _overlays.tracks[_overlays.own_robot]["pos"]
		var g := fwd * float(p[0]) + left * float(p[1])
		var expected := Vector2(o[0] - g.z, o[1] - g.x)  # Godot → ENU（東 = -z, 北 = -x）
		var e: Array = echoes[i]
		if Vector2(e[0], e[1]).distance_to(expected) > 0.05 or absf(float(e[2]) - float(p[3])) > 1.0e-3:
			problems.append("radar echo %d: %s expected (%.2f, %.2f, rcs %s)" % [i, e, expected.x, expected.y, p[3]])
	return problems

## Disturbance PDU の波浪パラメータが海面に反映され、箱庭時刻で進んでいるか
func _check_wave(expected: Dictionary) -> PackedStringArray:
	var problems := PackedStringArray()
	var actual: Dictionary = _ocean.get_wave_params()
	for key in expected.keys():
		if absf(float(actual.get(key, NAN)) - float(expected[key])) > 1.0e-9:
			problems.append("wave %s=%s expected %s" % [key, actual.get(key), expected[key]])
	if not _ocean.wave_active:
		problems.append("wave not active")
	if _ocean.sim_time_sec <= 0.0:
		problems.append("wave time not advancing with simulation (t=%s)" % _ocean.sim_time_sec)
	return problems

## 船体姿勢を独立計算した期待値と比較する。pos.angular.y は mujoco-runtime 規約（= -pitch）。
## vel（ENU [m/s]）を渡すと、位置は pos_pdu を始点とする等速直進の直線上にあるかで判定する（受信時刻が不明なため）。
func _check_pose(robot: String, pos_pdu: Dictionary, vel: Array = []) -> PackedStringArray:
	var problems := PackedStringArray()
	var root: Node3D = _fleet.ships.get(robot)
	if root == null:
		return PackedStringArray(["%s: ship not found" % robot])
	var lin: Dictionary = pos_pdu.get("linear", {})
	var ang: Dictionary = pos_pdu.get("angular", {})
	var flu_pos := Vector3(lin.get("x", 0.0), lin.get("y", 0.0), lin.get("z", 0.0))
	var roll := float(ang.get("x", 0.0))
	var pitch := -float(ang.get("y", 0.0))
	var yaw := float(ang.get("z", 0.0))

	var c: Basis = _fleet.FLU_TO_GODOT
	var expected_pos := c * flu_pos
	if not vel.is_empty():
		# 直線上の最近点を期待位置とする（進行方向の逆には戻らない）
		var dir := c * Vector3(vel[0], vel[1], 0.0)
		var s := maxf(0.0, (root.global_position - expected_pos).dot(dir) / maxf(dir.length_squared(), 1e-12))
		expected_pos += dir * s
	if root.global_position.distance_to(expected_pos) > POS_TOLERANCE_M:
		problems.append("%s: position %s expected %s" % [robot, root.global_position, expected_pos])

	var r_flu := Basis(Vector3(0, 0, 1), yaw) * Basis(Vector3(0, 1, 0), pitch) * Basis(Vector3(1, 0, 0), roll)
	var expected_basis := c * r_flu * c.inverse()
	var model: Node3D = root.get_node("Adapter/Model")
	var actual := model.global_basis
	for axis in 3:
		if actual[axis].distance_to(expected_basis[axis]) > ANGLE_TOLERANCE:
			problems.append("%s: model basis[%d] %s expected %s" % [robot, axis, actual[axis], expected_basis[axis]])
	# 物理的な向きの確認: 船首（glb -Z）、左舷（glb -X）
	var bow := -actual.z
	var port := -actual.x
	var expected_bow := c * (r_flu * Vector3(1, 0, 0))
	if bow.distance_to(expected_bow) > ANGLE_TOLERANCE:
		problems.append("%s: bow %s expected %s" % [robot, bow, expected_bow])
	if absf(pitch) > ANGLE_TOLERANCE and signf(bow.y) != -signf(pitch):
		problems.append("%s: pitch %+.3f must put bow %s, bow.y=%.4f" % [robot, pitch, "down" if pitch > 0 else "up", bow.y])
	if absf(roll) > ANGLE_TOLERANCE and signf(port.y) != signf(roll):
		problems.append("%s: roll %+.3f must put port side %s, port.y=%.4f" % [robot, roll, "up" if roll > 0 else "down", port.y])
	return problems

func _check_joints(robot: String, joint_values: Dictionary) -> PackedStringArray:
	var problems := PackedStringArray()
	var profile: Dictionary = _fleet._load_json("res://config/robot_sync/%s.profile.json" % robot)
	for mapping in profile.get("joint_mappings", []):
		var joint_name := str(mapping["joint_name"])
		if not joint_values.has(joint_name):
			continue
		var node: Node3D = _fleet.ships[robot].get_node(NodePath(str(mapping["node_path"])))
		var axis := {"x": Vector3(1, 0, 0), "y": Vector3(0, 1, 0), "z": Vector3(0, 0, 1)}[str(mapping["axis"])] as Vector3
		var expected := Basis(axis, float(mapping["sign"]) * float(joint_values[joint_name]))
		for i in 3:
			if node.basis[i].distance_to(expected[i]) > ANGLE_TOLERANCE:
				problems.append("%s: joint %s basis %s expected %s" % [robot, joint_name, node.basis, expected])
				break
	return problems

func _smoke_frames() -> int:
	var n := _arg_value("--smoke-frames=")
	return int(n) if n.is_valid_int() else SMOKE_FRAMES

func _has(arg: String) -> bool:
	return _args.has(arg)

func _arg_value(prefix: String) -> String:
	for arg in _args:
		if arg.begins_with(prefix):
			return arg.substr(prefix.length())
	return ""

func _fail(tag: String, detail: String) -> void:
	status_text = "%s: %s" % [tag, detail]
	printerr("%s\n  %s" % [tag, detail])
	get_tree().quit(1)
