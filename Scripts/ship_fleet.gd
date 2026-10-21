extends Node3D
## config/fleet.json の船舶を生成し、PDU 姿勢同期ノードを取り付ける（SHP-23 Step 2）。
##
## 階層: Ships/<robot>（ShipRoot, basis = FLU_TO_GODOT）/Adapter（basis = 逆変換）/Model（glb）
## ShipRoot は hakoniwa_robot_sync の target_root。apply_base_pose は ShipRoot を C·R（C = FLU_TO_GODOT,
## R = FLU 系の姿勢）にする。glb は Godot 座標（船首 -Z・上 +Y）で出力されているので、Adapter（C⁻¹）を
## 挟むと glb に掛かる回転は C·R·C⁻¹ となり、FLU の姿勢がそのまま Godot 上の姿勢になる。

const RobotSyncController = preload("res://addons/hakoniwa_robot_sync/scripts/robot_sync_controller.gd")
const TransformConverter = preload("res://addons/hakoniwa_robot_sync/scripts/transform_converter.gd")
const ShipPoseSync = preload("res://Scripts/ship_pose_sync.gd")

## 箱庭 FLU → Godot（x→-Z, y→-X, z→+Y）。hakoniwa_robot_sync の ros_position_to_godot と同じ対応。
## Basis(x_axis, y_axis, z_axis) は列ベクトル指定。
const FLU_TO_GODOT := Basis(Vector3(0, 0, -1), Vector3(-1, 0, 0), Vector3(0, 1, 0))
const COORDINATE_SYSTEM := {"position_rule": "hakoniwa_to_godot", "rotation_rule": "hakoniwa_to_godot"}

@export_file("*.json") var fleet_path: String = "res://config/fleet.json"

var fleet: Dictionary = {}
var ships: Dictionary = {}  # robot -> ShipRoot (Node3D)

## 船舶ノードを生成する。失敗時はエラー文字列を返す。
func build() -> String:
	fleet = _load_json(fleet_path)
	if fleet.is_empty():
		return "failed to load fleet: %s" % fleet_path
	for ship in fleet.get("ships", []):
		var robot := str(ship.get("robot", ""))
		var model := str(ship.get("model", ""))
		var scene: PackedScene = load("res://Models/%s/%s.glb" % [model, model])
		if scene == null:
			return "model not found: %s (run tools/sync_assets.ps1)" % model

		var root := Node3D.new()
		root.name = robot
		root.transform = Transform3D(FLU_TO_GODOT, Vector3.ZERO)
		var adapter := Node3D.new()
		adapter.name = "Adapter"
		adapter.transform = Transform3D(FLU_TO_GODOT.inverse(), Vector3.ZERO)
		var instance := scene.instantiate()
		instance.name = "Model"
		adapter.add_child(instance)
		root.add_child(adapter)
		add_child(root)
		ships[robot] = root
	return ""

func get_ship_configs() -> Array:
	return fleet.get("ships", [])

## PDU 未接続時の配置（fleet.json の preview_pose、PDU と同じ Twist 形式）。
func apply_preview_poses() -> void:
	for ship in get_ship_configs():
		apply_pose(str(ship["robot"]), ship.get("preview_pose", {}))

func apply_pose(robot: String, pos_pdu: Dictionary) -> void:
	TransformConverter.apply_base_pose(ships[robot], Transform3D(FLU_TO_GODOT, Vector3.ZERO), pos_pdu, COORDINATE_SYSTEM)

## 各船に PDU 同期ノードを付ける。joint_states を持つ船は hakoniwa_robot_sync、pos のみの船は ShipPoseSync。
## ShipRoot が初期姿勢（原点・FLU_TO_GODOT）のうちに呼ぶこと（同期ノードが初期変換として記録するため）。
func attach_sync(sim: Node) -> void:
	for ship in get_ship_configs():
		var robot := str(ship["robot"])
		var root: Node3D = ships[robot]
		if ship.has("joint_states_pdu"):
			var controller := RobotSyncController.new()
			controller.name = "%s_RobotSync" % robot
			controller.sim_node_path = sim.get_path()
			controller.target_root_path = root.get_path()
			controller.profile_path = "res://config/robot_sync/%s.profile.json" % robot
			add_child(controller)
		else:
			var pose_sync := ShipPoseSync.new()
			pose_sync.name = "%s_PoseSync" % robot
			pose_sync.sim = sim
			pose_sync.robot_name = robot
			pose_sync.pdu_name = str(ship.get("pos_pdu", "pos"))
			pose_sync.target_root = root
			add_child(pose_sync)

## robot_sync プロファイルの node_path が全て解決できるか検証する。未解決の node_path を返す。
func find_missing_joint_nodes() -> PackedStringArray:
	var missing := PackedStringArray()
	for ship in get_ship_configs():
		if not ship.has("joint_states_pdu"):
			continue
		var robot := str(ship["robot"])
		var profile := _load_json("res://config/robot_sync/%s.profile.json" % robot)
		if ships[robot].get_node_or_null(NodePath(str(profile.get("base_node_path", "")))) == null:
			missing.append("%s:%s" % [robot, profile.get("base_node_path", "")])
		for mapping in profile.get("joint_mappings", []):
			if ships[robot].get_node_or_null(NodePath(str(mapping["node_path"]))) == null:
				missing.append("%s:%s" % [robot, mapping["node_path"]])
	return missing

static func _load_json(path: String) -> Dictionary:
	if not FileAccess.file_exists(path):
		return {}
	var data = JSON.parse_string(FileAccess.get_file_as_string(path))
	return data if typeof(data) == TYPE_DICTIONARY else {}
