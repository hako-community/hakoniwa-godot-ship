extends SceneTree
## wave_spectrum.gd と Python/C++ 参照実装（reference.json）の一致検証。
##   Godot --headless --path <project> --script res://tests/unit/wave_parity/run_wave_parity.gd
## 成功時 WAVE_PARITY_OK / exit 0。

const WaveSpectrum = preload("res://Scripts/wave_spectrum.gd")
const REFERENCE := "res://tests/unit/wave_parity/reference.json"
const COMPONENT_FIELDS := ["omega", "k", "kx", "ky", "amp", "phase", "dir_rad", "choppiness"]
## 倍精度同士の比較。libm 実装差（数 ulp）を許容し、相対 1e-12 / 絶対 1e-12
const REL_TOL := 1.0e-12
const ABS_TOL := 1.0e-12

var _max_err := 0.0
var _problems := PackedStringArray()

func _init() -> void:
	var cases = JSON.parse_string(FileAccess.get_file_as_string(REFERENCE))
	if typeof(cases) != TYPE_ARRAY or cases.is_empty():
		printerr("WAVE_PARITY_FAILED reference not found: %s" % REFERENCE)
		quit(1)
		return
	var checked := 0
	for ci in cases.size():
		var case: Dictionary = cases[ci]
		var engine = WaveSpectrum.new()
		engine.initialize(case["params"])
		var comps: Array = case["components"]
		if engine.component_count() != comps.size():
			_problems.append("case%d: components %d expected %d" % [ci, engine.component_count(), comps.size()])
			continue
		for i in comps.size():
			for field in COMPONENT_FIELDS:
				_compare("case%d comp%d %s" % [ci, i, field], engine.get(field)[i], comps[i][field])
				checked += 1
		for s in case["samples"]:
			var surf: Dictionary = engine.get_surface(s["x"], s["y"], s["t"])
			var label := "case%d (%.1f,%.1f,t=%.1f)" % [ci, s["x"], s["y"], s["t"]]
			for field in ["elevation", "slope_x", "slope_y", "disp_x", "disp_y"]:
				_compare("%s %s" % [label, field], surf[field], s[field])
			for axis in 3:
				_compare("%s normal[%d]" % [label, axis], surf["normal"][axis], s["normal"][axis])
			_compare("%s get_elevation" % label, engine.get_elevation(s["x"], s["y"], s["t"]), s["elevation"])
			checked += 9
	if not _problems.is_empty():
		printerr("WAVE_PARITY_FAILED\n  " + "\n  ".join(_problems.slice(0, 20)))
		quit(1)
		return
	print("WAVE_PARITY_OK cases=%d values=%d max_abs_err=%s" % [cases.size(), checked, String.num_scientific(_max_err)])
	quit(0)

func _compare(label: String, actual: float, expected: float) -> void:
	var err := absf(actual - expected)
	_max_err = maxf(_max_err, err)
	if err > maxf(ABS_TOL, REL_TOL * absf(expected)):
		_problems.append("%s: %s expected %s (err %s)" % [label, String.num_scientific(actual), String.num_scientific(expected), String.num_scientific(err)])
