extends Node
## ビューアの起動モード管理と自己検証（SHP-23 Step 2〜3）。
##
## user args（`Godot ... -- <args>`）:
##   --smoke      オフラインで船隊生成・関節ノード解決・シーン向きを検証し HAKONIWA_GODOT_SHIP_BOOT_OK で終了
##   --hako       HakoniwaSimNode を初期化して PDU 同期を有効化（共有メモリ Z:/mmap_ship、conductor が必要）
##   --hako-start --hako に加え、初期化後にシミュレーション開始を要求
##   --sync-test  --hako-start に加え、tests/integration/pdu_sync/expected.json の値（船の姿勢・関節・波浪）を
##                受信・反映できたか検証し終了
##   --screenshot=<path>  --smoke と併用（ウィンドウ実行時のみ）。検証後の画面を PNG 保存

const SMOKE_FRAMES := 10
const SYNC_TEST_TIMEOUT_SEC := 30.0
const SYNC_EXPECTED_PATH := "res://tests/integration/pdu_sync/expected.json"
const POS_TOLERANCE_M := 1.0e-3
const ANGLE_TOLERANCE := 1.0e-3
const DisturbanceSync = preload("res://Scripts/disturbance_sync.gd")

@export var fleet_path: NodePath = ^"../Ships"
@export var sim_path: NodePath = ^"../Sim"
@export var camera_path: NodePath = ^"../Camera"
@export var sun_path: NodePath = ^"../Sun"
@export var ocean_path: NodePath = ^"../Ocean"

var status_text: String = "offline"

var _args := PackedStringArray()
var _fleet: Node3D
var _sim: Node
var _ocean: Node
var _offline_time := 0.0
var _frames := 0
var _sync_elapsed := 0.0
var _sync_expected: Dictionary = {}
var _sync_steps := 0

func _ready() -> void:
	_args = OS.get_cmdline_user_args()
	_fleet = get_node(fleet_path)
	_sim = get_node(sim_path)
	_ocean = get_node(ocean_path)

	var err: String = _fleet.build()
	if not err.is_empty():
		_fail("HAKONIWA_GODOT_SHIP_FLEET_FAILED", err)
		return

	if _has("--hako") or _has("--hako-start") or _has("--sync-test"):
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
	if _has("--smoke") and _frames == SMOKE_FRAMES:
		_run_smoke_checks()
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
		add_child(dist_sync)
	_sim.simulation_step.connect(func(_s, _w): _sync_steps += 1)
	var ret: int = _sim.initialize()
	if ret != 0:
		_fail("HAKONIWA_GODOT_SHIP_HAKO_INIT_FAILED", _sim.get_last_error_text())
		return
	status_text = "hakoniwa: initialized (%s)" % _sim.asset_name
	print("HAKONIWA_GODOT_SHIP_HAKO_INITIALIZED asset=%s" % _sim.asset_name)
	if _has("--hako-start") or _has("--sync-test"):
		ret = _sim.request_start()
		if ret != 0:
			_fail("HAKONIWA_GODOT_SHIP_HAKO_START_FAILED", _sim.get_last_error_text())
			return
		status_text = "hakoniwa: start requested"
	if _has("--sync-test"):
		_sync_expected = _fleet._load_json(SYNC_EXPECTED_PATH)
		if _sync_expected.is_empty():
			_fail("HAKONIWA_GODOT_SHIP_SYNC_FAILED", "expected values not found: %s" % SYNC_EXPECTED_PATH)

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
	# preview_pose が FLU→Godot 規約どおりに配置されているか
	for ship in _fleet.get_ship_configs():
		problems.append_array(_check_pose(str(ship["robot"]), ship.get("preview_pose", {})))
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
		problems.append_array(_check_pose(robot, expected.get("pos", {})))
		if expected.has("joint_states"):
			problems.append_array(_check_joints(robot, expected["joint_states"]))
	if _sync_expected.has("_wave"):
		problems.append_array(_check_wave(_sync_expected["_wave"]))
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
func _check_pose(robot: String, pos_pdu: Dictionary) -> PackedStringArray:
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
