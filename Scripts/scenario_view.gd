extends Node3D
## シナリオに合わせた表示（SHP-31）：SUT の航路、岸壁（デモ②）、海域の航路（デモ④⑤）、GNSS の測位と真の位置、霧。
##
## user args:
##   --scenario=<path>  デモのシナリオ JSON（hakoniwa-mujoco-ship demos/）。sut.route・quay を読む
##   --run-dir=<path>   実行の結果の置き場。traffic/area_enu.json（海域の航路の局所 ENU）があれば描く
## 座標は ENU（x 東・y 北）→ Godot（東 → -Z、北 → -X）。線は海面の少し上・深度テストなし。

const ROUTE_COLOR := Color(1.0, 1.0, 1.0, 0.35)
const LANE_COLOR := Color(1.0, 0.55, 0.85, 0.6)
const CENTER_COLOR := Color(1.0, 0.55, 0.85, 0.35)
const GNSS_COLOR := Color(1.0, 0.85, 0.2, 0.95)
const GNSS_FAULT_COLOR := Color(1.0, 0.25, 0.2, 0.95)
## 霧：視程 V [m] で消散係数 3 / V（コントラスト 5 %）。晴れはシーンの既定値。
## 俯瞰のカメラは自船から数 km 離れるので、そのままでは場面が見えない。カメラから自船までの濃さを最大 FOG_MAX_AT_SHIP に抑え、
## 視程は自船中心の円で示す（工学ビューア。見え方の再現は UE の主ビューアの役目）
const CLEAR_FOG_DENSITY := 5.0e-5
const FOG_MAX_AT_SHIP := 0.5
const VIS_COLOR := Color(0.85, 0.9, 1.0, 0.55)

var scenario: Dictionary = {}
var monitor: Node = null
var fleet: Node3D = null
var own_robot := "OwnShip"
var environment: Environment = null
var dist_sync: Node = null

var _lines: MeshInstance3D
var _gnss: MeshInstance3D
var _gnss_label: Label3D
var _vis: MeshInstance3D
var _vis_label: Label3D

func setup(scenario_path: String, run_dir: String) -> void:
	if not scenario_path.is_empty() and FileAccess.file_exists(scenario_path):
		var data = JSON.parse_string(FileAccess.get_file_as_string(scenario_path))
		if typeof(data) == TYPE_DICTIONARY:
			scenario = data
	_lines = _mesh_instance("ScenarioLines", 1)
	_gnss = _mesh_instance("GnssFix", 3)
	_vis = _mesh_instance("Visibility", 2)
	_vis_label = _label(VIS_COLOR)
	_gnss_label = _label(GNSS_COLOR)
	var mesh: ImmediateMesh = _lines.mesh
	mesh.clear_surfaces()
	mesh.surface_begin(Mesh.PRIMITIVE_LINES)
	var n := 0
	var route: Array = scenario.get("sut", {}).get("route", [])
	n += _polyline(mesh, route, ROUTE_COLOR, false)
	var area_path := run_dir.path_join("traffic/area_enu.json")
	if not run_dir.is_empty() and FileAccess.file_exists(area_path):
		var area = JSON.parse_string(FileAccess.get_file_as_string(area_path))
		if typeof(area) == TYPE_DICTIONARY:
			for r in area.get("routes", {}).values():
				n += _polyline(mesh, r.get("polygon", []), LANE_COLOR, true)
				n += _dashed(mesh, r.get("centerline", []), CENTER_COLOR)
	if n == 0:
		mesh.surface_add_vertex(Vector3.ZERO)
		mesh.surface_add_vertex(Vector3.ZERO)
	mesh.surface_end()
	if scenario.has("quay"):
		_build_quay(scenario["quay"])

func _process(_delta: float) -> void:
	_update_fog()
	_update_gnss()

## 霧：外乱 PDU の視程から。視程 9 km 未満なら自船中心に視程の円
func _update_fog() -> void:
	var mesh: ImmediateMesh = _vis.mesh
	mesh.clear_surfaces()
	_vis_label.visible = false
	if environment == null or dist_sync == null or float(dist_sync.visibility_m) <= 0.0:
		return
	var vis := float(dist_sync.visibility_m)
	if vis >= 9000.0:
		environment.fog_density = CLEAR_FOG_DENSITY
		return
	var density := 3.0 / vis
	var camera := get_viewport().get_camera_3d()
	if camera != null and fleet != null and fleet.ships.has(own_robot):
		var d := camera.global_position.distance_to(fleet.ships[own_robot].global_position)
		density = minf(density, -log(1.0 - FOG_MAX_AT_SHIP) / maxf(d, 1.0))
	environment.fog_density = maxf(CLEAR_FOG_DENSITY, density)
	if fleet == null or not fleet.ships.has(own_robot):
		return
	var o: Vector3 = fleet.ships[own_robot].global_position
	o.y = 1.5
	mesh.surface_begin(Mesh.PRIMITIVE_LINES)
	for k in 128:
		if k % 2 == 1:
			continue   # 破線
		var a0 := TAU * k / 128.0
		var a1 := TAU * (k + 1) / 128.0
		mesh.surface_set_color(VIS_COLOR)
		mesh.surface_add_vertex(o + Vector3(cos(a0), 0, sin(a0)) * vis)
		mesh.surface_set_color(VIS_COLOR)
		mesh.surface_add_vertex(o + Vector3(cos(a1), 0, sin(a1)) * vis)
	mesh.surface_end()
	_vis_label.visible = true
	_vis_label.position = o + Vector3(-vis, 10.0, 0.0)   # 北（-X）側
	_vis_label.text = "視程 %.1f km（霧）" % (vis / 1000.0)

## GNSS の測位（偽の位置を含む）と真の位置（自船）。離れていれば線で結ぶ
func _update_gnss() -> void:
	var mesh: ImmediateMesh = _gnss.mesh
	mesh.clear_surfaces()
	if monitor == null or monitor.gnss_fix.is_empty() or fleet == null or not fleet.ships.has(own_robot):
		_gnss_label.visible = false
		return
	var own: Vector3 = fleet.ships[own_robot].global_position
	var g := Vector3(-float(monitor.gnss_fix[2]), 2.0, -float(monitor.gnss_fix[1]))
	var row: Array = monitor.plant_rows.get(monitor.SENSOR_GNSS, [])
	var fault := not row.is_empty() and int(row[0]) == 3
	var color := GNSS_FAULT_COLOR if fault else GNSS_COLOR
	var err := Vector2(g.x - own.x, g.z - own.z).length()
	mesh.surface_begin(Mesh.PRIMITIVE_LINES)
	var r := 12.0
	for k in 32:
		var a0 := TAU * k / 32.0
		var a1 := TAU * (k + 1) / 32.0
		mesh.surface_set_color(color)
		mesh.surface_add_vertex(g + Vector3(cos(a0), 0, sin(a0)) * r)
		mesh.surface_set_color(color)
		mesh.surface_add_vertex(g + Vector3(cos(a1), 0, sin(a1)) * r)
	if err > r:
		mesh.surface_set_color(color)
		mesh.surface_add_vertex(g)
		mesh.surface_set_color(color)
		mesh.surface_add_vertex(Vector3(own.x, 2.0, own.z))
	mesh.surface_end()
	_gnss_label.visible = fault or err > 30.0
	_gnss_label.modulate = color
	_gnss_label.position = g + Vector3(0, 20, 0)
	_gnss_label.text = "GNSS（なりすまし中） 誤差 %.0f m" % err if fault else "GNSS 誤差 %.0f m" % err

func _label(color: Color) -> Label3D:
	var label := Label3D.new()
	label.billboard = BaseMaterial3D.BILLBOARD_ENABLED
	label.no_depth_test = true
	label.fixed_size = true
	label.pixel_size = 0.0008
	label.font_size = 26
	label.outline_size = 8
	label.modulate = color
	label.visible = false
	add_child(label)
	return label

func _mesh_instance(node_name: String, priority: int) -> MeshInstance3D:
	var inst := MeshInstance3D.new()
	inst.name = node_name
	inst.mesh = ImmediateMesh.new()
	inst.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
	inst.extra_cull_margin = 16384.0
	var mat := StandardMaterial3D.new()
	mat.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	mat.vertex_color_use_as_albedo = true
	mat.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA
	mat.no_depth_test = true
	mat.disable_fog = true
	mat.render_priority = priority
	inst.material_override = mat
	add_child(inst)
	return inst

static func _g(p) -> Vector3:
	return Vector3(-float(p[1]), 1.0, -float(p[0]))

func _polyline(mesh: ImmediateMesh, pts: Array, color: Color, closed: bool) -> int:
	if pts.size() < 2:
		return 0
	var count := pts.size() if closed else pts.size() - 1
	for i in count:
		mesh.surface_set_color(color)
		mesh.surface_add_vertex(_g(pts[i]))
		mesh.surface_set_color(color)
		mesh.surface_add_vertex(_g(pts[(i + 1) % pts.size()]))
	return count

func _dashed(mesh: ImmediateMesh, pts: Array, color: Color) -> int:
	var n := 0
	for i in range(0, pts.size() - 1, 2):
		mesh.surface_set_color(color)
		mesh.surface_add_vertex(_g(pts[i]))
		mesh.surface_set_color(color)
		mesh.surface_add_vertex(_g(pts[i + 1]))
		n += 1
	return n

## 岸壁：線分 p1→p2 の陸側に奥行き 20 m・高さ 3 m の箱、水側の面に防舷材（20 m 間隔）
func _build_quay(q: Dictionary) -> void:
	var p1 := Vector2(float(q["p1"][0]), float(q["p1"][1]))
	var p2 := Vector2(float(q["p2"][0]), float(q["p2"][1]))
	var along := p2 - p1
	var length := along.length()
	if length <= 0.0:
		return
	var dir := along / length
	var normal := Vector2(-dir.y, dir.x)   # 左手の法線
	# 自船の初期位置の側が水側
	for s in scenario.get("ships", []):
		if str(s.get("robot", "")) == str(scenario.get("own_ship", own_robot)):
			var o := Vector2(float(s["pose"]["x"]), float(s["pose"]["y"]))
			if (o - p1).dot(normal) > 0.0:
				normal = -normal   # normal を陸側へ
	var depth := 20.0
	var center := (p1 + p2) * 0.5 + normal * depth * 0.5
	var box := MeshInstance3D.new()
	box.name = "Quay"
	var bm := BoxMesh.new()
	bm.size = Vector3(depth, 4.0, length)   # ローカル Z が岸壁沿い
	box.mesh = bm
	var mat := StandardMaterial3D.new()
	mat.albedo_color = Color(0.62, 0.62, 0.6)
	box.material_override = mat
	add_child(box)
	box.position = Vector3(-center.y, 1.0, -center.x)
	box.basis = _basis_along(dir)
	var fender_mat := StandardMaterial3D.new()
	fender_mat.albedo_color = Color(0.08, 0.08, 0.08)
	var fender_m := float(q.get("fender_m", 1.0))
	var k := 10.0
	while k < length:
		var f := MeshInstance3D.new()
		var fm := BoxMesh.new()
		fm.size = Vector3(fender_m, 1.6, 1.6)
		f.mesh = fm
		f.material_override = fender_mat
		var fp := p1 + dir * k - normal * fender_m * 0.5
		add_child(f)
		f.position = Vector3(-fp.y, 1.4, -fp.x)
		f.basis = _basis_along(dir)
		k += 20.0

## ENU の向き dir をローカル Z に合わせる基底
static func _basis_along(dir: Vector2) -> Basis:
	var z := Vector3(-dir.y, 0.0, -dir.x).normalized()
	var x := Vector3.UP.cross(z).normalized()
	return Basis(x, Vector3.UP, z)
