extends Node
## 録画モード（SHP-35）：Godot の Movie Maker（`--write-movie <file> --fixed-fps <fps>`）と組み合わせ、1 コマごとに
## 箱庭のシミュレーションを「速さ × コマの時間」だけ進める。絵のコマとシミュレーションの時刻が 1 対 1 に対応するので、
## 計算の重さや他のアセットの待ちでコマが詰まったり同じ絵が続いたりしない（UE の -ShipMovie と同じ考え方、D39）。
##
## user arg `--movie=<schedule.json>`（シミュレーションの時刻で書く）:
##   {"fps": 25, "end_s": 900,
##    "speed": [[0, 4], [300, 1]],                       … その時刻からの速さ（整数倍。1 コマ = fps と delta の整数の歩）
##    "views": [[0, {"mode": "oblique", "distance": 2600, "elevation": 60, "azimuth": 0, "overlays": true}], …]
##              … mode は oblique（俯瞰）・top（真上）・chase（追跡）・bridge（船橋）。俯瞰どうしは 3 s かけて移り、視点が変わると切り替える。
##                overlays：工学オーバレイ（レーダの輪・危険な針路の扇など）を出すか（既定は俯瞰で出し、船の視点で出さない）
##    "captions": [[t0, t1, "テロップ"], …]}
## シミュレーションノード（hakoniwa_simulation_node.gd）の自動の進行は止め、ここから _tick_internal を呼ぶ（アドオンは無改変）。

const TRANSITION_S := 3.0
const WAIT_LIMIT_S := 120.0

var sim: Node = null
var camera: Camera3D = null
var hud: CanvasLayer = null
## レーダの掃引の層（速さ 2 倍を超えると 1 秒に何回転もしてちらついて見えるので隠す）
var sweep: Node3D = null
## 工学オーバレイ（船の視点では既定で隠す）と、シナリオの工学用の線（scenario_view.gd）
var overlays: Node3D = null
var scenario_view: Node = null
var schedule: Dictionary = {}

var _fps := 25.0
var _caption: Label
var _clock: Label
var _view_from: Dictionary = {}
var _view_to: Dictionary = {}
var _view_t0 := -INF
var _view_index := -1
var _started := false

func setup(path: String) -> String:
	if not FileAccess.file_exists(path):
		return "movie schedule not found: %s" % path
	var data = JSON.parse_string(FileAccess.get_file_as_string(path))
	if typeof(data) != TYPE_DICTIONARY:
		return "movie schedule is not a JSON object: %s" % path
	schedule = data
	_fps = float(schedule.get("fps", 25.0))
	return ""

func _ready() -> void:
	sim.set_physics_process(false)
	sim.set_process(false)
	_caption = Label.new()
	_caption.horizontal_alignment = HORIZONTAL_ALIGNMENT_CENTER
	_caption.autowrap_mode = TextServer.AUTOWRAP_WORD_SMART
	_caption.anchor_left = 0.08
	_caption.anchor_right = 0.92
	_caption.anchor_top = 1.0
	_caption.anchor_bottom = 1.0
	_caption.offset_top = -150.0
	_caption.offset_bottom = -40.0
	var font := SystemFont.new()
	font.font_names = PackedStringArray(["Yu Gothic UI", "Meiryo UI", "Meiryo", "MS Gothic"])
	font.font_weight = 700
	_caption.add_theme_font_override("font", font)
	_caption.add_theme_font_size_override("font_size", 34)
	_caption.add_theme_color_override("font_outline_color", Color(0, 0, 0, 0.9))
	_caption.add_theme_constant_override("outline_size", 10)
	hud.add_child(_caption)
	_clock = Label.new()
	_clock.add_theme_font_override("font", font)
	_clock.add_theme_font_size_override("font_size", 22)
	_clock.add_theme_color_override("font_outline_color", Color(0, 0, 0, 0.9))
	_clock.add_theme_constant_override("outline_size", 6)
	_clock.anchor_top = 1.0
	_clock.anchor_bottom = 1.0
	_clock.offset_left = 16.0
	_clock.offset_top = -36.0
	hud.add_child(_clock)
	# 開発用の状態の行（FPS・箱庭の状態）は動画に出さない
	var status := hud.get_node_or_null("Status")
	if status != null:
		status.visible = false

func _process(_delta: float) -> void:
	var delta_s: float = sim.delta_time_usec / 1.0e6
	if not _started:
		# 開始（箱庭の START と最初の 1 歩）まで待つ。この間のコマは開始前の絵
		_started = _advance_until(sim.get_simtime_usec() + 1, true)
		return
	var t: float = sim.get_simtime_usec() / 1.0e6
	if t >= float(schedule.get("end_s", INF)):
		print("HAKONIWA_GODOT_SHIP_MOVIE_DONE t=%.2f" % t)
		get_tree().quit(0)
		return
	var steps := maxi(1, int(round(_speed_at(t) / _fps / delta_s)))
	if not _advance_until(sim.get_simtime_usec() + steps * int(sim.delta_time_usec), false):
		printerr("HAKONIWA_GODOT_SHIP_MOVIE_STALLED t=%.2f" % t)
		get_tree().quit(1)
		return
	t = sim.get_simtime_usec() / 1.0e6
	if sweep != null:
		sweep.visible = _speed_at(t) <= 2.0 and (overlays == null or overlays.visible)
	_update_view(t)
	_update_caption(t)

## シミュレーションの時刻が target_usec に達するまで歩を進める（他のアセットを待つ間は少し眠る）
func _advance_until(target_usec: int, allow_wait_start: bool) -> bool:
	var waited := 0.0
	while sim.get_simtime_usec() < target_usec:
		if not sim._tick_internal(true):
			OS.delay_usec(200)
			waited += 0.0002
			if waited > WAIT_LIMIT_S:
				return false
			if allow_wait_start and waited > 0.05:
				return false   # 開始前：このコマは描いて次のコマでまた待つ
	return true

func _speed_at(t: float) -> float:
	var v := 1.0
	for pair in schedule.get("speed", []):
		if t + 1e-6 >= float(pair[0]):
			v = float(pair[1])
	return v

func _update_view(t: float) -> void:
	var views: Array = schedule.get("views", [])
	var idx := -1
	for i in views.size():
		if t + 1e-6 >= float(views[i][0]):
			idx = i
	if idx < 0 or camera == null:
		return
	if idx != _view_index:
		_view_index = idx
		var was_overview: bool = camera.mode == camera.Mode.OBLIQUE or camera.mode == camera.Mode.TOP
		_view_from = _current_view()
		_view_to = views[idx][1]
		# 船の視点から俯瞰へ戻るときは移らずに切り替える
		_view_t0 = t if idx > 0 and was_overview else -INF
	var mode_name := str(_view_to.get("mode", "top" if bool(_view_to.get("top", false)) else "oblique"))
	var modes := {"oblique": camera.Mode.OBLIQUE, "top": camera.Mode.TOP, "chase": camera.Mode.CHASE, "bridge": camera.Mode.BRIDGE}
	camera.mode = modes.get(mode_name, camera.Mode.OBLIQUE)
	var ship_view := mode_name == "chase" or mode_name == "bridge"
	var eng := bool(_view_to.get("overlays", not ship_view))
	if overlays != null:
		overlays.visible = eng
	if scenario_view != null:
		scenario_view.set_engineering_visible(eng)
	var targets := hud.get_node_or_null("Targets")
	if targets != null:
		targets.visible = eng   # 左上の CPA の一覧（工学の表示）
	if ship_view:
		return
	var a := clampf((t - _view_t0) / TRANSITION_S, 0.0, 1.0)
	a = a * a * (3.0 - 2.0 * a)
	for key in ["distance", "elevation", "azimuth"]:
		if _view_to.has(key):
			var v := lerpf(float(_view_from.get(key, _view_to[key])), float(_view_to[key]), a)
			match key:
				"distance":
					camera.distance_m = v
					camera.top_size_m = v * 2.0
				"elevation":
					camera.elevation_deg = v
				"azimuth":
					camera.azimuth_deg = v

func _current_view() -> Dictionary:
	return {"distance": camera.distance_m, "elevation": camera.elevation_deg, "azimuth": camera.azimuth_deg}

func _update_caption(t: float) -> void:
	var text := ""
	for c in schedule.get("captions", []):
		if t >= float(c[0]) and t < float(c[1]):
			text = str(c[2])
	_caption.text = text
	_clock.text = "シミュレーション時刻 %d:%02d　×%d" % [int(t) / 60, int(t) % 60, int(_speed_at(t))]
