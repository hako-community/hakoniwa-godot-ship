extends Node
## pos（geometry_msgs/Twist）のみを持つ他船の姿勢同期。
## HakoniwaRobotSyncController は joint_states を必須とするため、座標変換（HakoniwaTransformConverter）
## だけを再利用して pos を ShipRoot に反映する。

const TransformConverter = preload("res://addons/hakoniwa_robot_sync/scripts/transform_converter.gd")
const COORDINATE_SYSTEM := {"position_rule": "hakoniwa_to_godot", "rotation_rule": "hakoniwa_to_godot"}

var sim: Node = null
var robot_name: String = ""
var pdu_name: String = "pos"
var target_root: Node3D = null

var _initial: Transform3D
var _endpoint = null
var _typed_endpoint = null

func _ready() -> void:
	_initial = target_root.transform
	sim.simulation_ready.connect(_on_simulation_ready)
	sim.simulation_step.connect(_on_simulation_step)
	sim.simulation_reset.connect(_on_simulation_reset)

func _on_simulation_ready() -> void:
	_endpoint = sim.get_endpoint()
	if _endpoint == null:
		push_error("%s: %s" % [robot_name, sim.get_last_error_text()])
		return
	if _endpoint.create_pdu_lchannel(robot_name, pdu_name) != 0:
		push_error("%s: %s" % [robot_name, _endpoint.get_last_error_text()])
		return
	_typed_endpoint = sim.get_typed_endpoint(robot_name, pdu_name)
	if _typed_endpoint == null:
		push_error("%s: %s" % [robot_name, sim.get_last_error_text()])

func _on_simulation_step(_simtime_usec: int, _world_time_usec: int) -> void:
	if _typed_endpoint == null:
		return
	_endpoint.process_recv_events()
	var pos: Dictionary = _typed_endpoint.recv_dict()
	if not pos.is_empty():
		TransformConverter.apply_base_pose(target_root, _initial, pos, COORDINATE_SYSTEM)

func _on_simulation_reset() -> void:
	target_root.transform = _initial
