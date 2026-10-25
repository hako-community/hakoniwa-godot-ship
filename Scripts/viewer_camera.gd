extends Camera3D
## 自船追従カメラ（SHP-23 Step 4）。工学オーバレイ（数 km 規模）を見渡すための俯瞰ビュー。
##
## - oblique: 南側上空から北向きに見下ろす（画面上 ≒ 北）。マウスホイールで距離を変更
## - top    : 真上からの正射影・北が画面上（ノースアップ）。マウスホイールで表示範囲を変更
## - chase  : 自船の後ろ 160 m・右 60 m・上 45 m から船の少し上を見る（UE の追跡と同じ。向きは方位だけをなめらかに追う）
## - bridge : 船橋カメラの取り付け位置（bridge_mount_flu、船体座標）から前方を見る（船の揺れも一緒に）
## キー 1 追跡・2 船橋・3 俯瞰（UE と同じ）、C で俯瞰と真上の切替（起動時は `--camera=top|chase|bridge`）。
## focus（海面の詳細メッシュの追従先）は自船に置く。
## `--camera-distance=<m>` で起動時の距離、`--camera-elevation-deg=<deg>` で見下ろし角（oblique、既定 63.4°）を指定
## （船体と海面の近接確認・外観確認用）。

enum Mode { OBLIQUE, TOP, CHASE, BRIDGE }

@export var focus_path: NodePath
@export var mode: Mode = Mode.OBLIQUE
@export var distance_m: float = 3600.0
@export var top_size_m: float = 6000.0
@export var min_distance_m: float = 150.0
@export var max_distance_m: float = 12000.0
## oblique の見下ろし角 [deg]（既定は従来の (0.45, 0.9) × 距離 と同じ向き）
@export var elevation_deg: float = 63.435
## oblique のカメラの方位 [deg]（0：南から北を見る、90：西から東を見る。録画の予定 movie_driver.gd が変える）
@export var azimuth_deg: float = 0.0
## 船橋カメラの取り付け位置（船体座標 FLU [m]）。749GT の camera_bridge_front（−26.55, 0, 15.5、UE の fleet_ue.json）は
## Godot の glb では操舵室の閉じた箱の内側に入るので、前へ約 1 m 出した（UE の「箱の中ならレンズを前の面の外へ」と同じ安全策）
## 高さは前方の海を広く見せるため 2 m 上げた（船橋の窓の上の高さ、コンテナの段越しに前を見る）
@export var bridge_mount_flu: Vector3 = Vector3(-25.5, 0.0, 17.5)
@export var chase_offset_m: Vector3 = Vector3(-160.0, -60.0, 45.0)   # 後ろ・右（−左）・上
@export var chase_fov_deg: float = 60.0
@export var bridge_fov_deg: float = 75.0
@export var overview_fov_deg: float = 75.0

var target: Node3D = null

var _focus: Node3D

func _ready() -> void:
	_focus = get_node_or_null(focus_path) as Node3D
	var args := OS.get_cmdline_user_args()
	if args.has("--camera=top"):
		mode = Mode.TOP
	elif args.has("--camera=chase"):
		mode = Mode.CHASE
	elif args.has("--camera=bridge"):
		mode = Mode.BRIDGE
	for arg in args:
		if arg.begins_with("--camera-distance="):
			distance_m = clampf(arg.trim_prefix("--camera-distance=").to_float(), min_distance_m, max_distance_m)
		elif arg.begins_with("--camera-elevation-deg="):
			elevation_deg = clampf(arg.trim_prefix("--camera-elevation-deg=").to_float(), 2.0, 89.0)

func _unhandled_input(event: InputEvent) -> void:
	if event is InputEventKey and event.pressed and not event.echo:
		match event.keycode:
			KEY_C:
				mode = Mode.TOP if mode == Mode.OBLIQUE else Mode.OBLIQUE
			KEY_1:
				mode = Mode.CHASE
			KEY_2:
				mode = Mode.BRIDGE
			KEY_3:
				mode = Mode.OBLIQUE
	elif event is InputEventMouseButton and event.pressed:
		var k := 0.0
		if event.button_index == MOUSE_BUTTON_WHEEL_UP:
			k = 1.0 / 1.15
		elif event.button_index == MOUSE_BUTTON_WHEEL_DOWN:
			k = 1.15
		if k != 0.0:
			distance_m = clampf(distance_m * k, min_distance_m, max_distance_m)
			top_size_m = clampf(top_size_m * k, min_distance_m, max_distance_m * 2.0)

func _process(delta: float) -> void:
	if target == null:
		return
	var center := Vector3(target.global_position.x, 0.0, target.global_position.z)
	if _focus != null:
		_focus.global_position = center
	if mode == Mode.CHASE or mode == Mode.BRIDGE:
		_update_ship_view(delta)
		return
	_chase_valid = false
	fov = overview_fov_deg
	# Godot: 北 = -X, 東 = -Z（ship_fleet.gd の FLU_TO_GODOT）。南 = +X
	if mode == Mode.TOP:
		projection = PROJECTION_ORTHOGONAL
		size = top_size_m
		global_position = center + Vector3(0.0, max_distance_m * 0.5, 0.0)
		look_at(center, Vector3(-1.0, 0.0, 0.0))
	else:
		projection = PROJECTION_PERSPECTIVE
		var r := distance_m * 1.00623  # 既定の見下ろし角で従来の (0.45, 0.9) × 距離と一致
		var el := deg_to_rad(elevation_deg)
		var az := deg_to_rad(azimuth_deg)
		global_position = center + Vector3(r * cos(el) * cos(az), r * sin(el), r * cos(el) * sin(az))
		look_at(center, Vector3.UP)

var _chase_valid := false
var _chase_fwd := Vector3.ZERO

## 追跡・船橋。船のモデル（glb：船首 -Z・上 +Y・左舷 -X）の姿勢から
func _update_ship_view(delta: float) -> void:
	projection = PROJECTION_PERSPECTIVE
	var model := target.get_node_or_null("Adapter/Model") as Node3D
	if model == null:
		return
	var b := model.global_basis.orthonormalized()
	if mode == Mode.BRIDGE:
		fov = bridge_fov_deg
		# FLU（前 x・左 y・上 z）→ glb のローカル（-Z・-X・+Y）
		var local := Vector3(-bridge_mount_flu.y, bridge_mount_flu.z, -bridge_mount_flu.x)
		global_transform = Transform3D(b, model.global_position + b * local)
		_chase_valid = false
		return
	fov = chase_fov_deg
	# 位置は今の船に合わせ、向き（方位）だけをコマごとに寄せる（録画では 1 コマで数十秒進むので、位置を遅らせると船が画面から外れる）
	var fwd_now := Vector3(-b.z.x, 0.0, -b.z.z).normalized()
	_chase_fwd = fwd_now if not _chase_valid else _chase_fwd.lerp(fwd_now, clampf(delta * 2.0, 0.05, 0.3)).normalized()
	_chase_valid = true
	var fwd := _chase_fwd
	var left := Vector3.UP.cross(fwd)
	var pos := target.global_position
	global_position = pos + fwd * chase_offset_m.x + left * chase_offset_m.y + Vector3.UP * chase_offset_m.z
	look_at(pos + Vector3.UP * 8.0, Vector3.UP)
