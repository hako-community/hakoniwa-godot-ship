extends Camera3D
## 自船追従カメラ（SHP-23 Step 4）。工学オーバレイ（数 km 規模）を見渡すための俯瞰ビュー。
##
## - oblique: 南側上空から北向きに見下ろす（画面上 ≒ 北）。マウスホイールで距離を変更
## - top    : 真上からの正射影・北が画面上（ノースアップ）。マウスホイールで表示範囲を変更
## C キーで切替（起動時は user arg `--camera=top` でも指定可）。focus（海面の詳細メッシュの追従先）を注視点に置く。
## `--camera-distance=<m>` で起動時の距離（oblique）を指定（船体と海面の近接確認用）。

enum Mode { OBLIQUE, TOP }

@export var focus_path: NodePath
@export var mode: Mode = Mode.OBLIQUE
@export var distance_m: float = 3600.0
@export var top_size_m: float = 6000.0
@export var min_distance_m: float = 150.0
@export var max_distance_m: float = 12000.0

var target: Node3D = null

var _focus: Node3D

func _ready() -> void:
	_focus = get_node_or_null(focus_path) as Node3D
	var args := OS.get_cmdline_user_args()
	if args.has("--camera=top"):
		mode = Mode.TOP
	for arg in args:
		if arg.begins_with("--camera-distance="):
			distance_m = clampf(arg.trim_prefix("--camera-distance=").to_float(), min_distance_m, max_distance_m)

func _unhandled_input(event: InputEvent) -> void:
	if event is InputEventKey and event.pressed and not event.echo and event.keycode == KEY_C:
		mode = Mode.TOP if mode == Mode.OBLIQUE else Mode.OBLIQUE
	elif event is InputEventMouseButton and event.pressed:
		var k := 0.0
		if event.button_index == MOUSE_BUTTON_WHEEL_UP:
			k = 1.0 / 1.15
		elif event.button_index == MOUSE_BUTTON_WHEEL_DOWN:
			k = 1.15
		if k != 0.0:
			distance_m = clampf(distance_m * k, min_distance_m, max_distance_m)
			top_size_m = clampf(top_size_m * k, min_distance_m, max_distance_m * 2.0)

func _process(_delta: float) -> void:
	if target == null:
		return
	var center := Vector3(target.global_position.x, 0.0, target.global_position.z)
	if _focus != null:
		_focus.global_position = center
	# Godot: 北 = -X, 東 = -Z（ship_fleet.gd の FLU_TO_GODOT）。南 = +X
	if mode == Mode.TOP:
		projection = PROJECTION_ORTHOGONAL
		size = top_size_m
		global_position = center + Vector3(0.0, max_distance_m * 0.5, 0.0)
		look_at(center, Vector3(-1.0, 0.0, 0.0))
	else:
		projection = PROJECTION_PERSPECTIVE
		global_position = center + Vector3(distance_m * 0.45, distance_m * 0.9, 0.0)
		look_at(center, Vector3.UP)
