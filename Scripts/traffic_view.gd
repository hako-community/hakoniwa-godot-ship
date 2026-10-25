extends Node3D
## 海域の交通の船（OwnShip/traffic、SHP-28）を描く（SHP-31）。
##
## 1 点 = 1 隻（局所 ENU）。船長で船隊のモデルを選び、船長に合わせて縮尺する（UE の交通の描画と同じ考え方）。
## 位置は時刻印の値を対地速力・向きで表示時刻（物理時刻）まで進める。海面の上下動は付けない（運動学の再生のため）。
## タグ：AIS のある船は MMSI、無い船は NON-AIS（レーダにだけ映る）。

## 船長 [m] の上限 → モデル（上から順に当てはめる）
const MODEL_BY_LENGTH := [[25.0, "fishing_boat"], [75.0, "container_749gt"], [INF, "container_700teu"]]

var monitor: Node = null
## 表示時刻 [s] を返す Callable（物理時刻。無ければ箱庭時刻）
var clock: Callable

var _scenes: Dictionary = {}   # モデル名 → [PackedScene, 全長 m]
var _ships: Dictionary = {}    # mmsi → {node, model, label}
var _last_stamp := -1.0

func _process(_delta: float) -> void:
	if monitor == null or monitor.traffic_stamp < 0.0:
		return
	var t: float = clock.call() if clock.is_valid() else monitor.sim_time
	if monitor.traffic_stamp != _last_stamp:
		_last_stamp = monitor.traffic_stamp
		_sync_ships(monitor.traffic)
	for s in monitor.traffic:
		var entry: Dictionary = _ships.get(s["mmsi"], {})
		if entry.is_empty():
			continue
		# ENU の yaw（東から反時計回り）→ 真方位、対地速力で表示時刻まで進める
		var dt := clampf(t - float(monitor.traffic_stamp), -1.0, 1.0)
		var v := float(s["sog"]) * 0.514444
		var e := float(s["x"]) + v * cos(float(s["yaw"])) * dt
		var n := float(s["y"]) + v * sin(float(s["yaw"])) * dt
		var node: Node3D = entry["node"]
		node.position = Vector3(-n, 0.0, -e)
		# glb の船首は -Z（= 東）。真方位 b の向きは東から b − 90° 時計回り
		var bearing := 90.0 - rad_to_deg(float(s["yaw"]))
		node.basis = Basis(Vector3.UP, -deg_to_rad(bearing - 90.0)).scaled(Vector3.ONE * float(entry["scale"]))

func ship_count() -> int:
	return _ships.size()

## モデル名 → 全長 [m]（検証用）
func model_info() -> Dictionary:
	var out := {}
	for k in _scenes.keys():
		out[k] = _scenes[k][1] if not _scenes[k].is_empty() else 0.0
	return out

func _sync_ships(list: Array) -> void:
	var seen := {}
	for s in list:
		var mmsi: int = s["mmsi"]
		seen[mmsi] = true
		if _ships.has(mmsi):
			continue
		var model := _model_for(float(s["length"]))
		var info: Array = _scene(model)
		if info.is_empty():
			continue
		var node := Node3D.new()
		node.name = "Traffic_%d" % mmsi
		node.add_child(info[0].instantiate())
		var label := Label3D.new()
		label.billboard = BaseMaterial3D.BILLBOARD_ENABLED
		label.no_depth_test = true
		label.fixed_size = true
		label.pixel_size = 0.0007
		label.font_size = 24
		label.outline_size = 6
		label.modulate = Color(0.8, 0.95, 1.0) if s["ais"] else Color(1.0, 0.75, 0.4)
		label.text = ("MMSI %d" % mmsi) if s["ais"] else "NON-AIS"
		var scale := float(s["length"]) / maxf(float(info[1]), 1.0)
		label.position = Vector3(0.0, 18.0 / scale, 0.0)
		node.add_child(label)
		add_child(node)
		_ships[mmsi] = {"node": node, "model": model, "scale": scale}
	for mmsi in _ships.keys():
		if not seen.has(mmsi):
			_ships[mmsi]["node"].queue_free()
			_ships.erase(mmsi)

static func _model_for(length_m: float) -> String:
	for pair in MODEL_BY_LENGTH:
		if length_m <= pair[0]:
			return pair[1]
	return MODEL_BY_LENGTH.back()[1]

## [PackedScene, 全長 m]。全長はメッシュの AABB の前後（Z）
func _scene(model: String) -> Array:
	if _scenes.has(model):
		return _scenes[model]
	var scene: PackedScene = load("res://Models/%s/%s.glb" % [model, model])
	if scene == null:
		_scenes[model] = []
		return []
	var probe := scene.instantiate()
	var aabb := _aabb(probe, Transform3D.IDENTITY)
	probe.free()
	_scenes[model] = [scene, aabb.size.z]
	return _scenes[model]

static func _aabb(node: Node, xf: Transform3D) -> AABB:
	var out := AABB()
	var first := true
	var t := xf
	if node is Node3D:
		t = xf * (node as Node3D).transform
	if node is MeshInstance3D and (node as MeshInstance3D).mesh != null:
		out = t * (node as MeshInstance3D).mesh.get_aabb()
		first = false
	for child in node.get_children():
		var a := _aabb(child, t)
		if a.size == Vector3.ZERO:
			continue
		out = a if first else out.merge(a)
		first = false
	return out
