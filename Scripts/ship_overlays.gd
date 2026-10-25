extends Node3D
## 工学オーバレイ（SHP-23 Step 4）: 航跡線・AIS ベクトル／船名タグ・OZT 危険進路扇形・レーダ掃引／物標。
##
## - 入力は船の姿勢（ShipRoot、PDU 同期済み）と海面（潮位・Hs）、レーダ物標（set_radar_points()）。
##   対地速度は姿勢の時系列から最小二乗で推定する（物理側に COG/SOG の PDU が無いため。ADR: D16）。
## - CPA/TCPA/OZT は cpa_ozt.gd（hakoniwa-mujoco-ship cpa_ozt.hpp の移植）で自船と各他船について計算する。
## - 座標: ENU（x = 東, y = 北）。Godot へは 東 → -Z, 北 → -X（ship_fleet.gd の FLU_TO_GODOT と同じ）。
## - 描画は海面の少し上に置いた水平な帯・扇形で、深度テストなし（波に隠れない）。
##   線幅・点の大きさは画素単位（*_px）で指定し、自船位置での 1 画素あたりの長さに換算する（ズームしても見える）。
## - 性能: メッシュは原点基準のローカル座標で作り、毎フレームはノードの位置・向きだけを変える。
##   作り直すのは、レーダ同心円・クラッタ（縮尺・Hs の変化時）、掃引ビーム（縮尺の変化時）、物標（受信時）、
##   OZT とタグ文言（OZT 計算時 = update_interval_s ごと）、航跡（点の追加時）。AIS ベクトルのみ毎フレーム。

const CpaOzt = preload("res://Scripts/cpa_ozt.gd")
const NM := 1852.0
const KT := 0.514444
## 縮尺（1 画素あたりの長さ）がこの比率以上変わったら線幅依存のメッシュを作り直す
const RESCALE_RATIO := 1.15

var fleet: Node3D = null
var ocean: Node = null
var config: Dictionary = {}

var own_robot := ""
var time_sec := 0.0
## robot -> {pos: [e, n], heading_deg, vel: [ve, vn], samples: [[t, e, n], ...], trail: [[e, n], ...], last_trail_t}
var tracks: Dictionary = {}
## 他船 robot -> CpaOzt.compute_cpa / compute_ozt の結果（自船基準）
var cpa: Dictionary = {}
var ozt: Dictionary = {}
## 自船の現針路（COG）がいずれかの他船の危険進路に入っているか
var own_course_in_danger := false
## レーダ物標（自船ヘディングアップ水平座標 [x 前, y 左, z 上, RCS]）と、それを ENU に置いた位置 [e, n, RCS]
var radar_points: Array = []
var radar_echoes_enu: Array = []
var radar_sweep_rel_deg := 0.0
var summary_text := ""
## 直近の update() の所要時間 [ms]（HUD 表示・性能確認用）
var update_ms := 0.0

var _layers: Dictionary = {}  # name -> MeshInstance3D（mesh は ImmediateMesh）
var _labels: Dictionary = {}  # robot -> Label3D
var _has_vertices: Dictionary = {}  # ImmediateMesh -> 描画中の surface に頂点を置いたか
var _m_per_px := 1.0
var _last_ozt_t := -INF
var _radar_key := []  # 同心円・クラッタを作ったときの [縮尺, クラッタ半径, Hs]
var _sweep_scale := 0.0
var _trail_scale := 0.0
var _trail_dirty := true
var _echo_dirty := true
var _echo_scale := 0.0

## fleet.json の overlays 設定でレイヤとタグを作る。build() 済みの ship_fleet を渡すこと。
func setup(fleet_node: Node3D, ocean_node: Node) -> void:
	fleet = fleet_node
	ocean = ocean_node
	config = fleet.fleet.get("overlays", {})
	own_robot = str(config.get("own_ship", "OwnShip"))
	# 先に追加したものが下（render_priority 昇順）
	for layer in ["clutter", "radar", "sweep", "ozt", "trail", "ais", "echo"]:
		_add_layer(layer)
	for ship in fleet.get_ship_configs():
		var robot := str(ship["robot"])
		tracks[robot] = {"pos": [0.0, 0.0], "heading_deg": 0.0, "vel": [0.0, 0.0], "samples": [], "trail": [], "last_trail_t": -INF}
		var label := Label3D.new()
		label.name = "%s_Tag" % robot
		label.billboard = BaseMaterial3D.BILLBOARD_ENABLED
		label.no_depth_test = true
		label.fixed_size = true
		label.pixel_size = 0.0009
		label.font_size = 28
		label.outline_size = 8
		label.render_priority = 10
		label.outline_render_priority = 9
		label.vertical_alignment = VERTICAL_ALIGNMENT_BOTTOM
		add_child(label)
		_labels[robot] = label

## 箱庭時刻（またはオフライン時刻）t [s] で全オーバレイを更新する。
func update(t: float) -> void:
	var start_usec := Time.get_ticks_usec()
	time_sec = t
	for ship in fleet.get_ship_configs():
		_update_track(str(ship["robot"]), t)
	_update_radar(t)
	_m_per_px = _meters_per_pixel()
	var y := _overlay_y()
	for inst in _layers.values():
		inst.position.y = y

	# CPA/OZT は間引いて計算する。自船の速度が推定できる（標本 2 点以上）までは毎回計算し直す
	var ozt_cfg: Dictionary = config.get("ozt", {})
	if t - _last_ozt_t >= float(ozt_cfg.get("update_interval_s", 0.2)) or t < _last_ozt_t:
		_update_cpa_ozt()
		if tracks.has(own_robot) and tracks[own_robot]["samples"].size() >= 2:
			_last_ozt_t = t
		_draw_ozt()
		_update_label_texts()

	_place_own_ship_layers()
	_draw_radar_static()
	_draw_sweep()
	_draw_echoes()
	_draw_trails()
	_draw_ais()
	_place_labels(y)
	update_ms = (Time.get_ticks_usec() - start_usec) / 1000.0

## レーダ物標を受け取る。points は [x, y, z, rcs]（自船ヘディングアップ水平座標 [m], RCS [m²]）の配列。
func set_radar_points(points: Array) -> void:
	radar_points = points
	_echo_dirty = true

## JSON の数値は float になるため、MMSI は整数として表示する
static func format_mmsi(mmsi) -> String:
	return str(int(mmsi)) if typeof(mmsi) in [TYPE_INT, TYPE_FLOAT] else str(mmsi)

func get_label_text(robot: String) -> String:
	return _labels[robot].text if _labels.has(robot) else ""

func get_layer(layer_name: String) -> ImmediateMesh:
	return _layers[layer_name].mesh if _layers.has(layer_name) else null

# ---- 計算 -----------------------------------------------------------------

## Godot 座標 → ENU [東, 北]
static func godot_to_enu(g: Vector3) -> Array:
	return [-float(g.z), -float(g.x)]

## ENU [東, 北] と高さ → Godot 座標
static func enu_to_godot(e: float, n: float, y: float) -> Vector3:
	return Vector3(-n, y, -e)

## 真方位 θ [deg]（北から時計回り）の回転を Godot の Y 軸回りの基底で表す
static func bearing_basis(bearing_deg: float) -> Basis:
	return Basis(Vector3.UP, -deg_to_rad(bearing_deg))

func _update_track(robot: String, t: float) -> void:
	var root: Node3D = fleet.ships[robot]
	var tr: Dictionary = tracks[robot]
	tr["pos"] = godot_to_enu(root.global_position)
	var bow := -(root.get_node("Adapter/Model") as Node3D).global_basis.z
	var bow_enu := godot_to_enu(bow)
	tr["heading_deg"] = fposmod(rad_to_deg(atan2(bow_enu[0], bow_enu[1])), 360.0)

	var samples: Array = tr["samples"]
	if not samples.is_empty() and t < samples.back()[0]:
		# 時刻が戻った（リセット）: 履歴を捨てる
		samples.clear()
		tr["trail"].clear()
		tr["last_trail_t"] = -INF
		_trail_dirty = true
	if samples.is_empty() or t > samples.back()[0]:
		samples.append([t, tr["pos"][0], tr["pos"][1]])
	var window := float(config.get("velocity_window_s", 5.0))
	while samples.size() > 2 and t - samples[0][0] > window:
		samples.pop_front()
	tr["vel"] = _fit_velocity(samples)

	var trail_cfg: Dictionary = config.get("trail", {})
	if t - float(tr["last_trail_t"]) >= float(trail_cfg.get("sample_interval_s", 1.0)):
		tr["last_trail_t"] = t
		tr["trail"].append(tr["pos"].duplicate())
		var max_points := int(trail_cfg.get("max_points", 900))
		while tr["trail"].size() > max_points:
			tr["trail"].pop_front()
		_trail_dirty = true

## 位置の時系列 [t, e, n] から等速直線の速度 [ve, vn] を最小二乗で求める
static func _fit_velocity(samples: Array) -> Array:
	var count := samples.size()
	if count < 2:
		return [0.0, 0.0]
	var mt := 0.0
	var me := 0.0
	var mn := 0.0
	for s in samples:
		mt += s[0]
		me += s[1]
		mn += s[2]
	mt /= count
	me /= count
	mn /= count
	var stt := 0.0
	var ste := 0.0
	var stn := 0.0
	for s in samples:
		var dt: float = s[0] - mt
		stt += dt * dt
		ste += dt * (s[1] - me)
		stn += dt * (s[2] - mn)
	if stt <= 0.0:
		return [0.0, 0.0]
	return [ste / stt, stn / stt]

func _update_cpa_ozt() -> void:
	cpa.clear()
	ozt.clear()
	own_course_in_danger = false
	if not tracks.has(own_robot):
		return
	var own: Dictionary = tracks[own_robot]
	var own_speed := _speed(own)
	var own_course := _course(own)
	var cfg: Dictionary = config.get("ozt", {})
	var safe_m := float(cfg.get("safe_domain_m", 926.0))
	var horizon_s := float(cfg.get("time_horizon_s", 900.0))
	for robot in tracks.keys():
		if robot == own_robot:
			continue
		var tgt: Dictionary = tracks[robot]
		cpa[robot] = CpaOzt.compute_cpa(own["pos"], own["vel"], deg_to_rad(own_course), tgt["pos"], tgt["vel"], safe_m, horizon_s)
		ozt[robot] = CpaOzt.compute_ozt(own["pos"], own_speed, tgt["pos"], tgt["vel"], safe_m, horizon_s)
		if own_speed > 0.1 and CpaOzt.is_course_dangerous(ozt[robot]["danger_courses_deg"], own_course):
			own_course_in_danger = true

static func _speed(tr: Dictionary) -> float:
	return Vector2(tr["vel"][0], tr["vel"][1]).length()

## 対地針路 COG [deg]。停止中は船首方位
static func _course(tr: Dictionary) -> float:
	if _speed(tr) < 0.05:
		return tr["heading_deg"]
	return fposmod(rad_to_deg(atan2(tr["vel"][0], tr["vel"][1])), 360.0)

func _update_radar(t: float) -> void:
	var radar: Dictionary = config.get("radar", {})
	radar_sweep_rel_deg = fposmod(t * float(radar.get("rotation_rpm", 24.0)) * 6.0, 360.0)
	radar_echoes_enu.clear()
	if not tracks.has(own_robot):
		return
	var own: Dictionary = tracks[own_robot]
	var h := deg_to_rad(own["heading_deg"])
	# ヘディングアップ座標（x 前, y 左）→ ENU: 前 = (sin h, cos h), 左 = (-cos h, sin h)
	for p in radar_points:
		var x: float = p[0]
		var y: float = p[1]
		radar_echoes_enu.append([
			own["pos"][0] + x * sin(h) - y * cos(h),
			own["pos"][1] + x * cos(h) + y * sin(h),
			float(p[3]),
		])

## 物理の marine_radar.hpp と同じシークラッタ範囲: min(STC 距離, (1 + 0.8 Hs) × 1200 m)
func sea_clutter_radius_m() -> float:
	var radar: Dictionary = config.get("radar", {})
	if not ocean.wave_active or not bool(radar.get("sea_clutter_enabled", true)):
		return 0.0
	var hs := float(ocean.get_wave_params().get("hs_m", 0.0))
	return minf(float(radar.get("stc_range_nm", 3.0)) * NM, (1.0 + hs * 0.8) * 1200.0)

# ---- 描画 -----------------------------------------------------------------

func _add_layer(layer_name: String) -> void:
	var inst := MeshInstance3D.new()
	inst.name = layer_name.capitalize()
	inst.mesh = ImmediateMesh.new()
	inst.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
	inst.extra_cull_margin = 16384.0
	var mat := StandardMaterial3D.new()
	mat.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	mat.vertex_color_use_as_albedo = true
	mat.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA
	mat.no_depth_test = true
	mat.disable_fog = true   # 霧（デモ③）でもオーバレイは見える
	mat.cull_mode = BaseMaterial3D.CULL_DISABLED
	mat.render_priority = _layers.size()
	inst.material_override = mat
	add_child(inst)
	_layers[layer_name] = inst

func _overlay_y() -> float:
	return float(ocean.get_wave_params().get("tide_level_m", 0.0)) + float(config.get("height_above_sea_m", 0.5))

## 自船位置での 1 画素あたりの長さ [m]（カメラが無ければ 1）
func _meters_per_pixel() -> float:
	var camera := get_viewport().get_camera_3d()
	var height := float(get_viewport().get_visible_rect().size.y)
	if camera == null or height <= 0.0 or not tracks.has(own_robot):
		return 1.0
	if camera.projection == Camera3D.PROJECTION_ORTHOGONAL:
		return camera.size / height
	var p: Array = tracks[own_robot]["pos"]
	var d := camera.global_position.distance_to(enu_to_godot(p[0], p[1], 0.0))
	return 2.0 * d * tan(deg_to_rad(camera.fov) * 0.5) / height

func _px(px: float) -> float:
	return px * _m_per_px

static func _rescaled(built_scale: float, current: float) -> bool:
	return built_scale <= 0.0 or current / built_scale > RESCALE_RATIO or built_scale / current > RESCALE_RATIO

## 自船中心のレイヤ（同心円・クラッタ・掃引・物標）を自船位置・向きへ置く
func _place_own_ship_layers() -> void:
	if not tracks.has(own_robot):
		return
	var own: Dictionary = tracks[own_robot]
	var at := enu_to_godot(own["pos"][0], own["pos"][1], 0.0)
	for layer in ["clutter", "radar"]:
		_layers[layer].position = Vector3(at.x, _layers[layer].position.y, at.z)
	var sweep: MeshInstance3D = _layers["sweep"]
	sweep.position = Vector3(at.x, sweep.position.y, at.z)
	sweep.basis = bearing_basis(own["heading_deg"] + radar_sweep_rel_deg)
	var echo: MeshInstance3D = _layers["echo"]
	echo.position = Vector3(at.x, echo.position.y, at.z)
	echo.basis = bearing_basis(own["heading_deg"])

## レーダ同心円とシークラッタ（自船中心のローカル座標・ノースアップ）
func _draw_radar_static() -> void:
	var clutter_r := sea_clutter_radius_m()
	var hs := float(ocean.get_wave_params().get("hs_m", 0.0)) if clutter_r > 0.0 else 0.0
	if not _radar_key.is_empty() and not _rescaled(_radar_key[0], _m_per_px) and _radar_key[1] == clutter_r and _radar_key[2] == hs:
		return
	_radar_key = [_m_per_px, clutter_r, hs]
	var radar: Dictionary = config.get("radar", {})
	var range_m := float(radar.get("display_range_nm", 1.5)) * NM
	var ring_m := float(radar.get("ring_interval_nm", 0.5)) * NM
	var o := Vector2.ZERO
	var mesh: ImmediateMesh = _begin("radar")
	var r := ring_m
	while r <= range_m + 1.0:
		_ring(mesh, o, r, _px(1.5), Color(0.3, 1.0, 0.5, 0.45))
		r += ring_m
	_end(mesh)
	# シークラッタ: 範囲は marine_radar.hpp と同じ。濃さは Hs に比例し、STC と同様に距離とともに弱める
	mesh = _begin("clutter")
	var cr := minf(clutter_r, range_m)
	if cr > 0.0:
		var bands := 12
		for k in bands:
			var a := clampf(hs / 4.0, 0.1, 1.0) * 0.3 * pow(1.0 - float(k) / bands, 2.0)
			_fan(mesh, o, 0.0, 360.0, cr * k / bands, cr * (k + 1) / bands, Color(0.95, 0.95, 0.6, a))
	_end(mesh)

## アンテナビーム（水平ビーム幅）と残光。ローカル座標で真北向きに作り、ノードを掃引方位へ回す
func _draw_sweep() -> void:
	if not _rescaled(_sweep_scale, _m_per_px):
		return
	_sweep_scale = _m_per_px
	var radar: Dictionary = config.get("radar", {})
	var range_m := float(radar.get("display_range_nm", 1.5)) * NM
	var bw := float(radar.get("horizontal_beamwidth_deg", 1.8))
	var trail_deg := float(radar.get("afterglow_deg", 40.0))
	var mesh: ImmediateMesh = _begin("sweep")
	var steps := 20
	for k in steps:
		var a0 := -bw * 0.5 - trail_deg * float(k + 1) / steps
		var a1 := -bw * 0.5 - trail_deg * float(k) / steps
		_fan(mesh, Vector2.ZERO, a0, a1, 0.0, range_m, Color(0.3, 1.0, 0.5, 0.18 * (1.0 - float(k) / steps)))
	# ビーム幅 1.8° は遠方でも数画素なので、最低 2 画素の幅を持たせる
	var half := maxf(bw * 0.5, rad_to_deg(_px(1.0) / range_m))
	_fan(mesh, Vector2.ZERO, -half, half, 0.0, range_m, Color(0.6, 1.0, 0.7, 0.85))
	_end(mesh)

## レーダ物標。ローカル座標はヘディングアップ（前 = 北, 左 = 西）で作り、ノードを船首方位へ回す
func _draw_echoes() -> void:
	if not _echo_dirty and not _rescaled(_echo_scale, _m_per_px):
		return
	_echo_dirty = false
	_echo_scale = _m_per_px
	var mesh: ImmediateMesh = _begin("echo")
	for p in radar_points:
		var rcs: float = maxf(p[3], 0.01)
		# RCS に応じた大きさ（1 m² で 3 px、10⁴ m² で 30 px）と色（黄 → 赤）
		var radius := _px(clampf(3.0 * pow(rcs, 0.25), 2.0, 30.0))
		var heat := clampf(log(rcs) / log(10.0) / 4.0, 0.0, 1.0)
		_disc(mesh, Vector2(-float(p[1]), float(p[0])), radius, Color(1.0, lerpf(0.95, 0.25, heat), 0.1, 0.9))
	_end(mesh)

## OZT: 危険進路ごとに、自船から衝突予測点（速力 × TCPA）までの扇と衝突予測点。計算時の自船位置を原点とする
func _draw_ozt() -> void:
	var mesh: ImmediateMesh = _begin("ozt")
	var own: Dictionary = tracks.get(own_robot, {})
	if not own.is_empty():
		var o := Vector2(own["pos"][0], own["pos"][1])
		var inst: MeshInstance3D = _layers["ozt"]
		var at := enu_to_godot(o.x, o.y, 0.0)
		inst.position = Vector3(at.x, inst.position.y, at.z)
		var range_m := float(config.get("ozt", {}).get("display_range_m", 3000.0))
		var speed := _speed(own)
		var half := CpaOzt.COURSE_STEP_DEG * 0.5
		for robot in ozt.keys():
			var s: Dictionary = ozt[robot]
			var courses: Array = s["danger_courses_deg"]
			for i in courses.size():
				var r := minf(speed * float(s["tcpa_s"][i]), range_m)
				_fan(mesh, Vector2.ZERO, float(courses[i]) - half, float(courses[i]) + half, 0.0, r, Color(1.0, 0.1, 0.1, 0.28))
			for pt in s["danger_polygon"]:
				var q := Vector2(pt[0], pt[1]) - o
				if q.length() <= range_m:
					_disc(mesh, q, _px(2.5), Color(1.0, 0.2, 0.2, 0.9))
	_end(mesh)

func _draw_trails() -> void:
	if not _trail_dirty and not _rescaled(_trail_scale, _m_per_px):
		return
	_trail_dirty = false
	_trail_scale = _m_per_px
	var width := _px(float(config.get("trail", {}).get("width_px", 2.0)))
	var mesh: ImmediateMesh = _begin("trail")
	for robot in tracks.keys():
		var pts: Array = tracks[robot]["trail"]
		var base := Color(0.2, 1.0, 0.4) if robot == own_robot else Color(1.0, 0.85, 0.3)
		for i in range(1, pts.size()):
			# 古いほど薄く
			var c := base
			c.a = lerpf(0.15, 0.9, float(i) / pts.size())
			_ribbon(mesh, Vector2(pts[i - 1][0], pts[i - 1][1]), Vector2(pts[i][0], pts[i][1]), width, c)
	_end(mesh)

func _draw_ais() -> void:
	var ais_cfg: Dictionary = config.get("ais", {})
	var seconds := float(ais_cfg.get("vector_minutes", 6.0)) * 60.0
	var width := _px(float(ais_cfg.get("width_px", 3.0)))
	var mesh: ImmediateMesh = _begin("ais")
	for ship in fleet.get_ship_configs():
		var robot := str(ship["robot"])
		var tr: Dictionary = tracks[robot]
		var own := robot == own_robot
		# 自船の速度ベクトルは常に描く（危険進路上なら赤）。他船は AIS 搭載船のみ
		if not own and not bool(ship.get("ais", {}).get("has_ais", false)):
			continue
		var color := Color(0.3, 0.8, 1.0, 0.95)
		if own:
			color = Color(1.0, 0.2, 0.2, 0.95) if own_course_in_danger else Color(0.2, 1.0, 0.4, 0.95)
		elif cpa.has(robot) and cpa[robot]["collision_risk"]:
			color = Color(1.0, 0.35, 0.2, 0.95)
		var p := Vector2(tr["pos"][0], tr["pos"][1])
		var tip := p + Vector2(tr["vel"][0], tr["vel"][1]) * seconds
		_ribbon(mesh, p, tip, width, color)
		# 矢じり
		var d := tip - p
		if d.length() > width * 4.0:
			var u := d.normalized()
			var side := Vector2(-u.y, u.x) * width * 2.5
			var back := tip - u * width * 6.0
			_tri(mesh, tip, back + side, back - side, color)
	_end(mesh)

func _place_labels(y: float) -> void:
	var height := float(config.get("label_height_m", 25.0))
	for robot in _labels.keys():
		var p: Array = tracks[robot]["pos"]
		_labels[robot].position = enu_to_godot(p[0], p[1], y + height)

func _update_label_texts() -> void:
	var lines_out := PackedStringArray()
	for ship in fleet.get_ship_configs():
		var robot := str(ship["robot"])
		var tr: Dictionary = tracks[robot]
		var ais: Dictionary = ship.get("ais", {})
		var label: Label3D = _labels[robot]
		var motion := "SOG %.1f kt  COG %03.0f°" % [_speed(tr) / KT, _course(tr)]
		var lines := PackedStringArray()
		if robot == own_robot:
			lines.append("%s (OWN)" % ais.get("name", robot))
			lines.append(motion)
			label.modulate = Color(1.0, 0.4, 0.4) if own_course_in_danger else Color(0.5, 1.0, 0.6)
		else:
			var has_ais := bool(ais.get("has_ais", false))
			if has_ais:
				lines.append(str(ais.get("name", robot)))
				lines.append("MMSI %s" % format_mmsi(ais.get("mmsi", "-")))
				lines.append(motion)
			else:
				lines.append("NON-AIS (radar only)")
			var risk := false
			if cpa.has(robot):
				var m: Dictionary = cpa[robot]
				risk = m["collision_risk"]
				lines.append("CPA %.2f NM  TCPA %.1f min" % [m["cpa_distance_m"] / NM, m["tcpa_seconds"] / 60.0])
				var arc_text := PackedStringArray()
				for arc in ozt[robot]["danger_arcs"]:
					arc_text.append("%03d-%03d°" % [arc[0], arc[1]])
				var ozt_text := " ".join(arc_text) if not arc_text.is_empty() else "none"
				lines.append("OZT %s" % ozt_text)
				lines_out.append("%-18s CPA %5.2f NM  TCPA %5.1f min  OZT %s%s" % [
					robot, m["cpa_distance_m"] / NM, m["tcpa_seconds"] / 60.0, ozt_text, "  !" if risk else ""])
			if risk:
				label.modulate = Color(1.0, 0.35, 0.3)
			elif has_ais:
				label.modulate = Color(0.6, 0.9, 1.0)
			else:
				label.modulate = Color(0.75, 0.75, 0.75)
		label.text = "\n".join(lines)
	var head := "own course %s  |  radar echoes %d  |  overlay %.2f ms" % [
		"IN OZT" if own_course_in_danger else "clear", radar_points.size(), update_ms]
	summary_text = head + "\n" + "\n".join(lines_out)

# ---- 図形（ENU 平面上の三角形列。y はノード側で与える） ---------------

func _begin(layer_name: String) -> ImmediateMesh:
	var mesh: ImmediateMesh = _layers[layer_name].mesh
	mesh.clear_surfaces()
	mesh.surface_begin(Mesh.PRIMITIVE_TRIANGLES)
	return mesh

func _end(mesh: ImmediateMesh) -> void:
	# 頂点が無い surface_end はエラーになるため、空のときは不可視の三角形を 1 つ置く
	if not _has_vertices.get(mesh, false):
		mesh.surface_set_color(Color(0, 0, 0, 0))
		for i in 3:
			mesh.surface_add_vertex(Vector3.ZERO)
	_has_vertices.erase(mesh)
	mesh.surface_end()

func _tri(mesh: ImmediateMesh, a: Vector2, b: Vector2, c: Vector2, color: Color) -> void:
	mesh.surface_set_color(color)
	mesh.surface_add_vertex(Vector3(-a.y, 0.0, -a.x))
	mesh.surface_add_vertex(Vector3(-b.y, 0.0, -b.x))
	mesh.surface_add_vertex(Vector3(-c.y, 0.0, -c.x))
	_has_vertices[mesh] = true

func _ribbon(mesh: ImmediateMesh, a: Vector2, b: Vector2, width: float, color: Color) -> void:
	var d := b - a
	if d.length() < 1e-6:
		return
	var side := Vector2(-d.y, d.x).normalized() * width * 0.5
	_tri(mesh, a - side, a + side, b + side, color)
	_tri(mesh, a - side, b + side, b - side, color)

## 真方位 a0〜a1 [deg]、半径 r0〜r1 の扇（環扇）
func _fan(mesh: ImmediateMesh, o: Vector2, a0: float, a1: float, r0: float, r1: float, color: Color) -> void:
	var segments := maxi(1, int(ceil(absf(a1 - a0) / 3.0)))
	for i in segments:
		var t0 := deg_to_rad(lerpf(a0, a1, float(i) / segments))
		var t1 := deg_to_rad(lerpf(a0, a1, float(i + 1) / segments))
		var u0 := Vector2(sin(t0), cos(t0))
		var u1 := Vector2(sin(t1), cos(t1))
		if r0 <= 0.0:
			_tri(mesh, o, o + u0 * r1, o + u1 * r1, color)
		else:
			_tri(mesh, o + u0 * r0, o + u0 * r1, o + u1 * r1, color)
			_tri(mesh, o + u0 * r0, o + u1 * r1, o + u1 * r0, color)

func _ring(mesh: ImmediateMesh, o: Vector2, r: float, width: float, color: Color) -> void:
	_fan(mesh, o, 0.0, 360.0, r - width * 0.5, r + width * 0.5, color)

func _disc(mesh: ImmediateMesh, o: Vector2, r: float, color: Color) -> void:
	for i in 8:
		var a0 := TAU * i / 8.0
		var a1 := TAU * (i + 1) / 8.0
		_tri(mesh, o, o + Vector2(cos(a0), sin(a0)) * r, o + Vector2(cos(a1), sin(a1)) * r, color)
