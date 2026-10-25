extends Node
## センサ・通信の状態の画面（SHP-31、ADR D40）が使う PDU を受信して保持する。
##
## fleet.json の monitor（robot と PDU 名）:
##   sensor_status   std_msgs/Float64MultiArray  物理（生成側の真値。注入中の異常も）
##   sut_sensor_use  std_msgs/Float64MultiArray  SUT（判断に使っているか・健全と判断したか）
##     どちらも data = [version, stamp_s, rows, row_width, row...]、row = [sensor_id, state, last_update_s, rate_hz, v0..v3]
##     （hakoniwa-mujoco-ship include/ship/plant/sensor_status.hpp）
##   autonomy_status std_msgs/String（SUT の JSON：al・odd・mrs・reason・nav・crs・spd・tgt・gps）
##   foc_command     std_msgs/String（FOC の JSON：cmd・crs・spd・at）
##   gnss            nav_msgs/Odometry（局所 ENU の測位。なりすまし中は偽の位置）
##   ais             std_msgs/String（!AIVDM。1 PDU = 1 文）
##   traffic         sensor_msgs/PointCloud2（海域の交通 1 点 = 1 隻：x・y・yaw・length・beam・sog・mmsi・ais）
##   comm_status     std_msgs/Float64MultiArray（SUT の通信ゲートウェイ、SHP-32：健全性・遅延・回線ごとの状態。
##                   並びは hakoniwa-mujoco-ship include/ship/comm/ship_shore_link.hpp の status()）
##   autonomy_status_shore std_msgs/String（陸に届いた自動化レベルの報告＝FOC が見ているもの）
##   lidar           sensor_msgs/PointCloud2（離着桟 LiDAR、SHP-33：船体座標 x 前・y 左・z 上、岸壁に当たった点）
##   camera          sensor_msgs/PointCloud2（カメラの検出、SHP-34：x・y（船体座標、推定の距離）・size・camera（0 可視光 / 1 赤外線））

const RadarEchoSync = preload("res://Scripts/radar_echo_sync.gd")
const SENSOR_RADAR := 1
const SENSOR_AIS := 2
const SENSOR_GNSS := 3
const SENSOR_GYRO := 4
const SENSOR_ECHO := 5
const SENSOR_LIDAR := 6
const SENSOR_CAMERA_VIS := 7
const SENSOR_CAMERA_IR := 8
const SENSOR_NAV := 9

var sim: Node = null
var robot_name := "OwnShip"
var pdus: Dictionary = {}   # 役割 → PDU 名（fleet.json の monitor）

## sensor_id → [state, last_update_s, rate_hz, v0, v1, v2, v3]
var plant_rows: Dictionary = {}
var sut_rows: Dictionary = {}
var plant_stamp := -1.0
var sut_stamp := -1.0
var autonomy: Dictionary = {}
var autonomy_count := 0
var autonomy_time := -1.0
var foc: Dictionary = {}
var foc_count := 0
var foc_time := -1.0
## GNSS の測位 [stamp_s, east, north]
var gnss_fix: Array = []
var ais_count := 0
var ais_last := ""
## 交通の船 [{x, y, yaw, length, beam, sog, mmsi, ais}] と時刻印 [s]
var traffic: Array = []
var traffic_stamp := -1.0
## 船陸の通信路（SHP-32）。comm_status の中身（parse_comm）。無ければ空（理想の回線）
var comm: Dictionary = {}
## FOC が見ている自動化レベル（陸に届いた報告）
var autonomy_shore: Dictionary = {}
## 離着桟 LiDAR の点 [[x, y, z, mount], ...]（船体座標）と時刻印
var lidar_points: Array = []
var lidar_stamp := -1.0
## カメラの検出 [{x, y, size, camera}]
var camera_dets: Array = []
## 直近の受信時の箱庭時刻 [s]
var sim_time := 0.0

var _endpoint = null
var _typed: Dictionary = {}   # 役割 → typed endpoint
var _last_raw: Dictionary = {}

func _ready() -> void:
	sim.simulation_ready.connect(_on_simulation_ready)
	sim.simulation_step.connect(_on_simulation_step)

func _on_simulation_ready() -> void:
	_endpoint = sim.get_endpoint()
	if _endpoint == null:
		push_error("monitor: no endpoint")
		return
	for role in pdus.keys():
		var pdu := str(pdus[role])
		if _endpoint.create_pdu_lchannel(robot_name, pdu) != 0:
			push_error("%s/%s: %s" % [robot_name, pdu, sim.get_last_error_text()])
			continue
		_typed[role] = sim.get_typed_endpoint(robot_name, pdu)

func _on_simulation_step(simtime_usec: int, _world_time_usec: int) -> void:
	if _typed.is_empty():
		return
	sim_time = simtime_usec / 1.0e6
	_endpoint.process_recv_events()
	var d := _recv("sensor_status")
	if not d.is_empty():
		var parsed := parse_status(d.get("data", []))
		if not parsed.is_empty():
			plant_stamp = parsed[0]
			plant_rows = parsed[1]
	d = _recv("sut_sensor_use")
	if not d.is_empty():
		var parsed := parse_status(d.get("data", []))
		if not parsed.is_empty():
			sut_stamp = parsed[0]
			sut_rows = parsed[1]
	var s := _recv_string("autonomy_status")
	if not s.is_empty():
		var j = JSON.parse_string(s)
		if typeof(j) == TYPE_DICTIONARY:
			autonomy = j
			autonomy_count += 1
			autonomy_time = sim_time
	s = _recv_string("foc_command")
	if not s.is_empty():
		var j = JSON.parse_string(s)
		if typeof(j) == TYPE_DICTIONARY:
			foc = j
			foc_count += 1
			foc_time = sim_time
	d = _recv("lidar")
	if not d.is_empty():
		var st: Dictionary = d.get("header", {}).get("stamp", {})
		lidar_stamp = float(st.get("sec", 0)) + float(st.get("nanosec", 0)) * 1.0e-9
		lidar_points = RadarEchoSync.parse_points(d)
	d = _recv("camera")
	if not d.is_empty():
		camera_dets = parse_fields(d, ["x", "y", "size", "camera"])
	d = _recv("comm_status")
	if not d.is_empty():
		comm = parse_comm(d.get("data", []))
	s = _recv_string("autonomy_status_shore")
	if not s.is_empty():
		var j = JSON.parse_string(s)
		if typeof(j) == TYPE_DICTIONARY:
			autonomy_shore = j
	s = _recv_string("ais")
	if not s.is_empty():
		ais_count += 1
		ais_last = s
	d = _recv("gnss")
	if not d.is_empty():
		var st: Dictionary = d.get("header", {}).get("stamp", {})
		var stamp := float(st.get("sec", 0)) + float(st.get("nanosec", 0)) * 1.0e-9
		var pos: Dictionary = d.get("pose", {}).get("pose", {}).get("position", {})
		if stamp > 0.0 and (gnss_fix.is_empty() or stamp != gnss_fix[0]):
			gnss_fix = [stamp, float(pos.get("x", 0.0)), float(pos.get("y", 0.0))]
	d = _recv("traffic")
	if not d.is_empty():
		var st: Dictionary = d.get("header", {}).get("stamp", {})
		var stamp := float(st.get("sec", 0)) + float(st.get("nanosec", 0)) * 1.0e-9
		if stamp != traffic_stamp:
			traffic_stamp = stamp
			traffic = parse_traffic(d)

## 前回と同じ中身なら空を返す（latest バッファは同じ値を返し続けるため）
func _recv(role: String) -> Dictionary:
	if not _typed.has(role):
		return {}
	var d: Dictionary = _typed[role].recv_dict()
	if d.is_empty():
		return {}
	var key := hash(d)
	if _last_raw.get(role, 0) == key:
		return {}
	_last_raw[role] = key
	return d

func _recv_string(role: String) -> String:
	var d := _recv(role)
	return str(d.get("data", "")) if not d.is_empty() else ""

## sensor_status / sut_sensor_use の data を [stamp_s, {sensor_id: [state, last, rate, v0..v3]}] にする（形が違えば空）
static func parse_status(data) -> Array:
	var a: Array = Array(data)
	if a.size() < 4 or int(a[0]) != 1:
		return []
	var n := int(a[2])
	var w := int(a[3])
	if w < 8 or a.size() < 4 + n * w:
		return []
	var rows := {}
	for i in n:
		var b := 4 + i * w
		rows[int(a[b])] = [int(a[b + 1]), float(a[b + 2]), float(a[b + 3]), float(a[b + 4]), float(a[b + 5]), float(a[b + 6]), float(a[b + 7])]
	return [float(a[1]), rows]

## FLOAT32 のフィールドだけの PointCloud2 を辞書の配列に（名前が無ければ空）
static func parse_fields(cloud: Dictionary, names: Array) -> Array:
	var offsets := {}
	for f in cloud.get("fields", []):
		offsets[str(f.get("name", ""))] = int(f.get("offset", 0))
	for n in names:
		if not offsets.has(n):
			return []
	var data = cloud.get("data", PackedByteArray())
	var bytes: PackedByteArray = data if data is PackedByteArray else PackedByteArray(data)
	var step := int(cloud.get("point_step", 0))
	var count := int(cloud.get("width", 0)) * maxi(1, int(cloud.get("height", 1)))
	if step <= 0 or bytes.size() < count * step:
		return []
	var out: Array = []
	for i in count:
		var e := {}
		for n in names:
			e[n] = bytes.decode_float(i * step + offsets[n])
		out.append(e)
	return out

## comm_status の data を辞書に（形が違えば空）
static func parse_comm(data) -> Dictionary:
	var a: Array = Array(data)
	if a.size() < 5 or int(a[0]) != 1:
		return {}
	var n := int(a[2])
	var w := int(a[3])
	var extra := int(a[4])
	var base := 5 + extra
	if extra < 12 or w < 8 or a.size() < base + n * w:
		return {}
	var links: Array = []
	for i in n:
		var b := base + i * w
		links.append({"leg": int(a[b]), "index": int(a[b + 1]), "type": int(a[b + 2]), "available": a[b + 3] > 0.5,
			"active": float(a[b + 4]), "latency_ms": float(a[b + 5]), "bandwidth_mbps": float(a[b + 6]), "priority": int(a[b + 7])})
	return {"stamp": float(a[1]), "healthy": a[5] > 0.5, "up": a[6] > 0.5, "up_ms": float(a[7]), "down_ms": float(a[8]),
		"down_for": float(a[9]), "down_sent": int(a[10]), "down_delivered": int(a[11]), "down_dropped": int(a[12]),
		"up_delivered": int(a[13]), "last_up_sent": float(a[14]), "last_up_delivered": float(a[15]), "last_down_sent": float(a[16]),
		"links": links}

## 交通の PointCloud2（point_step 32 B：FLOAT32 x y yaw length beam sog、UINT32 mmsi、FLOAT32 ais）
static func parse_traffic(cloud: Dictionary) -> Array:
	var offsets := {}
	for f in cloud.get("fields", []):
		offsets[str(f.get("name", ""))] = int(f.get("offset", 0))
	for key in ["x", "y", "yaw", "length", "sog", "mmsi", "ais"]:
		if not offsets.has(key):
			return []
	var data = cloud.get("data", PackedByteArray())
	var bytes: PackedByteArray = data if data is PackedByteArray else PackedByteArray(data)
	var step := int(cloud.get("point_step", 0))
	var count := int(cloud.get("width", 0)) * maxi(1, int(cloud.get("height", 1)))
	if step <= 0 or bytes.size() < count * step:
		return []
	var out: Array = []
	for i in count:
		var b := i * step
		out.append({
			"x": bytes.decode_float(b + offsets["x"]),
			"y": bytes.decode_float(b + offsets["y"]),
			"yaw": bytes.decode_float(b + offsets["yaw"]),
			"length": bytes.decode_float(b + offsets["length"]),
			"beam": bytes.decode_float(b + offsets["beam"]) if offsets.has("beam") else 0.0,
			"sog": bytes.decode_float(b + offsets["sog"]),
			"mmsi": bytes.decode_u32(b + offsets["mmsi"]),
			"ais": bytes.decode_float(b + offsets["ais"]) > 0.5,
		})
	return out
