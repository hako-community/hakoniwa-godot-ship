extends Node
## レーダ物標 PDU（sensor_msgs/PointCloud2）を受信してオーバレイへ渡す（SHP-23 Step 4）。
##
## 契約（ADR: D16）: OwnShip/radar_points の各点は自船ヘディングアップ水平座標（x 前, y 左, z 上）[m]、
## フィールド x / y / z / intensity は FLOAT32（datatype 7）・リトルエンディアン、intensity = 推定 RCS [m²]。

const FLOAT32 := 7

var sim: Node = null
var overlays: Node = null
var robot_name: String = ""
var pdu_name: String = "radar_points"

var _endpoint = null
var _typed_endpoint = null

func _ready() -> void:
	sim.simulation_ready.connect(_on_simulation_ready)
	sim.simulation_step.connect(_on_simulation_step)

func _on_simulation_ready() -> void:
	_endpoint = sim.get_endpoint()
	if _endpoint == null or _endpoint.create_pdu_lchannel(robot_name, pdu_name) != 0:
		push_error("%s/%s: %s" % [robot_name, pdu_name, sim.get_last_error_text()])
		return
	_typed_endpoint = sim.get_typed_endpoint(robot_name, pdu_name)

func _on_simulation_step(_simtime_usec: int, _world_time_usec: int) -> void:
	if _typed_endpoint == null:
		return
	_endpoint.process_recv_events()
	var cloud: Dictionary = _typed_endpoint.recv_dict()
	if not cloud.is_empty():
		overlays.set_radar_points(parse_points(cloud))

## PointCloud2 の辞書表現を [x, y, z, intensity] の配列にする。必須フィールドが無ければ空配列。
static func parse_points(cloud: Dictionary) -> Array:
	var offsets := {}
	for f in cloud.get("fields", []):
		if int(f.get("datatype", 0)) == FLOAT32:
			offsets[str(f.get("name", ""))] = int(f.get("offset", 0))
	for key in ["x", "y", "z"]:
		if not offsets.has(key):
			return []
	if bool(cloud.get("is_bigendian", false)):
		return []
	var data = cloud.get("data", PackedByteArray())
	var bytes: PackedByteArray = data if data is PackedByteArray else PackedByteArray(data)
	var step := int(cloud.get("point_step", 0))
	var count := int(cloud.get("width", 0)) * maxi(1, int(cloud.get("height", 1)))
	if step <= 0 or bytes.size() < count * step:
		return []
	var points: Array = []
	for i in count:
		var base := i * step
		points.append([
			bytes.decode_float(base + offsets["x"]),
			bytes.decode_float(base + offsets["y"]),
			bytes.decode_float(base + offsets["z"]),
			bytes.decode_float(base + offsets["intensity"]) if offsets.has("intensity") else 0.0,
		])
	return points
