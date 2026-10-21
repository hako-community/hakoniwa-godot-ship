extends Node
## 海上外乱 PDU（hako_msgs/Disturbance）から波浪パラメータを取り出し海面へ反映する（SHP-23 Step 3）。
##
## 取り出し規則は ShipHydroPlugin::parse_from_custom() / update_wave_engine_if_needed() と同一:
##   d_user_custom[1].data[4] = water_depth_m
##   d_user_custom[2].data    = [hs_m, tp_s, wave_dir_rad, gamma, seed]
## 物理プラグインは潮位（d_user_custom[1].data[5]）を波面に入れていないため、ここでも tide_level_m = 0 とする。

var sim: Node = null
var ocean: Node = null
var robot_name: String = ""
var pdu_name: String = "maritime_disturbance"
var last_params: Dictionary = {}

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

func _on_simulation_step(simtime_usec: int, _world_time_usec: int) -> void:
	ocean.set_time(simtime_usec / 1.0e6)
	if _typed_endpoint == null:
		return
	_endpoint.process_recv_events()
	var params := parse_wave_params(_typed_endpoint.recv_dict())
	if not params.is_empty() and params != last_params:
		last_params = params
		ocean.set_wave_params(params)

## Disturbance の辞書表現から波浪パラメータを取り出す。波浪成分が無ければ空辞書。
static func parse_wave_params(disturbance: Dictionary) -> Dictionary:
	var customs: Array = disturbance.get("d_user_custom", [])
	if customs.size() < 3:
		return {}
	var c1: Array = customs[1].get("data", [])
	var c2: Array = customs[2].get("data", [])
	if c1.size() < 5 or c2.size() < 5:
		return {}
	return {
		"hs_m": float(c2[0]),
		"tp_s": float(c2[1]),
		"wave_dir_rad": float(c2[2]),
		"gamma": float(c2[3]),
		"seed": int(c2[4]),
		"water_depth_m": float(c1[4]),
		"tide_level_m": 0.0,
	}
