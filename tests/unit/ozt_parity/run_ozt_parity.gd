extends SceneTree
## cpa_ozt.gd と C++ 実装（cpa_ozt.hpp → reference.json）の一致検証、および danger_arcs() の検証。
##   Godot --headless --path <project> --script res://tests/unit/ozt_parity/run_ozt_parity.gd
## 成功時 OZT_PARITY_OK / exit 0。

const CpaOzt = preload("res://Scripts/cpa_ozt.gd")
const REFERENCE := "res://tests/unit/ozt_parity/reference.json"
const CPA_FIELDS := ["cpa_distance_m", "tcpa_seconds", "bcr_distance_m", "bct_seconds"]
const CPA_FLAGS := ["bow_cross_ahead", "is_closing", "collision_risk"]
## 倍精度同士の比較。libm 実装差（数 ulp）を許容
const REL_TOL := 1.0e-12
const ABS_TOL := 1.0e-9

var _max_err := 0.0
var _problems := PackedStringArray()

func _init() -> void:
	var ref = JSON.parse_string(FileAccess.get_file_as_string(REFERENCE))
	if typeof(ref) != TYPE_DICTIONARY:
		printerr("OZT_PARITY_FAILED reference not found: %s" % REFERENCE)
		quit(1)
		return
	var checked := 0
	for i in ref["cpa"].size():
		var c: Dictionary = ref["cpa"][i]
		var m := CpaOzt.compute_cpa(c["own_pos"], c["own_vel"], c["heading_rad"], c["tgt_pos"], c["tgt_vel"],
				c["cpa_alarm_dist_m"], c["tcpa_alarm_time_s"])
		for f in CPA_FIELDS:
			_compare("cpa%d %s" % [i, f], m[f], c[f])
		for f in CPA_FLAGS:
			if m[f] != c[f]:
				_problems.append("cpa%d %s: %s expected %s" % [i, f, m[f], c[f]])
		checked += CPA_FIELDS.size() + CPA_FLAGS.size()

	var wrap_cases := 0
	for i in ref["ozt"].size():
		var c: Dictionary = ref["ozt"][i]
		var s := CpaOzt.compute_ozt(c["own_pos"], c["own_speed_mps"], c["tgt_pos"], c["tgt_vel"],
				c["safe_domain_m"], c["time_horizon_s"])
		if s["has_danger"] != c["has_danger"]:
			_problems.append("ozt%d has_danger: %s expected %s" % [i, s["has_danger"], c["has_danger"]])
		_compare("ozt%d min_course" % i, s["min_danger_course_deg"], c["min_danger_course_deg"])
		_compare("ozt%d max_course" % i, s["max_danger_course_deg"], c["max_danger_course_deg"])
		var poly: Array = s["danger_polygon"]
		var expected_poly: Array = c["danger_polygon"]
		if poly.size() != expected_poly.size():
			_problems.append("ozt%d polygon size %d expected %d" % [i, poly.size(), expected_poly.size()])
			continue
		for k in poly.size():
			_compare("ozt%d poly%d.x" % [i, k], poly[k][0], expected_poly[k][0])
			_compare("ozt%d poly%d.y" % [i, k], poly[k][1], expected_poly[k][1])
		checked += 3 + poly.size() * 2
		# 区間化: 全危険進路を過不足なく覆い、区間数は min/max からは分からない真北跨ぎも 1 区間にまとめる
		var courses: Array = s["danger_courses_deg"]
		var covered: Array = []
		for arc in CpaOzt.danger_arcs(courses):
			var d: int = arc[0]
			while true:
				covered.append(d)
				if d == arc[1]:
					break
				d = (d + CpaOzt.COURSE_STEP_DEG) % 360
		covered.sort()
		if covered != courses:
			_problems.append("ozt%d danger_arcs do not cover courses exactly" % i)
		if s["has_danger"] and courses.front() == 0 and courses.back() == 358 and courses.size() < 180:
			wrap_cases += 1
			if CpaOzt.danger_arcs(courses).size() != _count_runs_circular(courses):
				_problems.append("ozt%d wrap arc not merged" % i)
	_check_arc_examples()

	if not _problems.is_empty():
		printerr("OZT_PARITY_FAILED\n  " + "\n  ".join(_problems.slice(0, 20)))
		quit(1)
		return
	print("OZT_PARITY_OK cpa=%d ozt=%d (north-wrap=%d) values=%d max_abs_err=%s" % [
		ref["cpa"].size(), ref["ozt"].size(), wrap_cases, checked, String.num_scientific(_max_err)])
	quit(0)

func _check_arc_examples() -> void:
	var cases := [
		[[], []],
		[[10, 12, 14, 30], [[10, 14], [30, 30]]],
		[[0, 2, 4, 350, 352, 354, 356, 358], [[350, 4]]],
		[[0, 2, 100, 358], [[358, 2], [100, 100]]],
	]
	for c in cases:
		var arcs := CpaOzt.danger_arcs(c[0])
		if arcs != c[1]:
			_problems.append("danger_arcs(%s) = %s expected %s" % [c[0], arcs, c[1]])
	if not CpaOzt.is_course_dangerous([0, 2, 358], 359.2) or CpaOzt.is_course_dangerous([0, 2, 358], 4.2):
		_problems.append("is_course_dangerous binning")

## 円環上の連続区間数（独立計算）
func _count_runs_circular(courses: Array) -> int:
	var runs := 0
	for c in courses:
		if not courses.has((int(c) - CpaOzt.COURSE_STEP_DEG + 360) % 360):
			runs += 1
	return runs

func _compare(label: String, actual: float, expected: float) -> void:
	var err := absf(actual - expected)
	_max_err = maxf(_max_err, err)
	if err > maxf(ABS_TOL, REL_TOL * absf(expected)):
		_problems.append("%s: %s expected %s (err %s)" % [label, String.num_scientific(actual), String.num_scientific(expected), String.num_scientific(err)])
