extends SceneTree
## GPU（wave_common.gdshaderinc, 単精度）と CPU（wave_spectrum.gd, 倍精度）の波面一致検証。
## GPU 描画が必要なためウィンドウ実行（--headless 不可）:
##   Godot --path <project> --script res://tests/unit/wave_shader_parity/run_wave_shader_parity.gd
## 成功時 WAVE_SHADER_PARITY_OK / exit 0。

const WaveSpectrum = preload("res://Scripts/wave_spectrum.gd")
const PROBES := 64
const ETA_RANGE := 64.0
const SLOPE_RANGE := 4.0
const SANITY := 0.123456
## 単精度 GPU の許容誤差（位相は中心・時刻を CPU で畳み込み済み、相対座標は ±1100 m 以内）
const ETA_TOL_M := 5.0e-3
const SLOPE_TOL := 2.0e-3
## 長時間・遠方条件: 物理プラグインと同じ 32 成分、浅水、t ≈ 1 時間、中心は原点から離す
const PARAMS := {"hs_m": 3.0, "tp_s": 9.0, "wave_dir_rad": 0.6, "gamma": 3.3, "seed": 7, "water_depth_m": 40.0, "num_components": 32}
const CENTER_E := 5234.0
const CENTER_N := -2567.0
const TIME_SEC := 3605.25

var _viewport: SubViewport
var _wave = WaveSpectrum.new()
var _points := PackedVector2Array()
var _frames := 0

func _initialize() -> void:
	if DisplayServer.get_name() == "headless":
		printerr("WAVE_SHADER_PARITY_FAILED requires a GPU window (run without --headless)")
		quit(1)
		return
	_wave.initialize(PARAMS)
	# 決定論的に散らした検証点（中心からの相対 ENU、±1100 m）
	var rng := RandomNumberGenerator.new()
	rng.seed = 20261021
	_points.append(Vector2.ZERO)
	while _points.size() < PROBES:
		_points.append(Vector2(rng.randf_range(-1100.0, 1100.0), rng.randf_range(-1100.0, 1100.0)))

	var material := ShaderMaterial.new()
	material.shader = load("res://tests/unit/wave_shader_parity/wave_probe.gdshader")
	material.set_shader_parameter("wave_count", _wave.component_count())
	material.set_shader_parameter("wave_k_amp", _wave.shader_k_amp())
	material.set_shader_parameter("wave_phase", _wave.shader_phases(CENTER_E, CENTER_N, TIME_SEC))
	material.set_shader_parameter("wave_tide", 0.0)
	material.set_shader_parameter("probe_count", PROBES)
	material.set_shader_parameter("probe_points", _points)
	material.set_shader_parameter("eta_range", ETA_RANGE)
	material.set_shader_parameter("slope_range", SLOPE_RANGE)
	material.set_shader_parameter("sanity_value", SANITY)

	_viewport = SubViewport.new()
	_viewport.size = Vector2i(PROBES, 4)
	_viewport.render_target_update_mode = SubViewport.UPDATE_ALWAYS
	_viewport.transparent_bg = false
	var rect := ColorRect.new()
	rect.size = Vector2(PROBES, 4)
	rect.material = material
	_viewport.add_child(rect)
	root.add_child(_viewport)

func _process(_delta: float) -> bool:
	_frames += 1
	if _frames < 4:
		return false
	var image := _viewport.get_texture().get_image()
	var sanity := _decode(image.get_pixel(0, 3), SLOPE_RANGE)
	if absf(sanity - SANITY) > 1.0e-6:
		printerr("WAVE_SHADER_PARITY_FAILED readback encoding broken: sanity %s expected %s (format=%d)" % [sanity, SANITY, image.get_format()])
		quit(1)
		return true
	var max_eta := 0.0
	var max_slope := 0.0
	var problems := PackedStringArray()
	for i in PROBES:
		var surf: Dictionary = _wave.get_surface(CENTER_E + _points[i].x, CENTER_N + _points[i].y, TIME_SEC)
		var eta_err := absf(_decode(image.get_pixel(i, 0), ETA_RANGE) - surf["elevation"])
		var slope_err := maxf(absf(_decode(image.get_pixel(i, 1), SLOPE_RANGE) - surf["slope_x"]),
			absf(_decode(image.get_pixel(i, 2), SLOPE_RANGE) - surf["slope_y"]))
		max_eta = maxf(max_eta, eta_err)
		max_slope = maxf(max_slope, slope_err)
		if eta_err > ETA_TOL_M or slope_err > SLOPE_TOL:
			problems.append("probe %d %s: eta_err=%s slope_err=%s (eta=%s)" % [i, _points[i], eta_err, slope_err, surf["elevation"]])
	if not problems.is_empty():
		printerr("WAVE_SHADER_PARITY_FAILED\n  " + "\n  ".join(problems.slice(0, 10)))
		quit(1)
		return true
	print("WAVE_SHADER_PARITY_OK probes=%d max_eta_err_m=%s max_slope_err=%s" % [PROBES, String.num_scientific(max_eta), String.num_scientific(max_slope)])
	quit(0)
	return true

## 8bit RGB 3 チャンネルの 24bit 固定小数を復号（Color は 0..1 の float なので 255 倍して丸める）
static func _decode(c: Color, range: float) -> float:
	var u := roundi(c.r * 255.0) * 65536 + roundi(c.g * 255.0) * 256 + roundi(c.b * 255.0)
	return (float(u) / 16777215.0 - 0.5) * range
