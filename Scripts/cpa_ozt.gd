extends RefCounted
## CPA / TCPA / BCR / OZT 計算（SHP-23 Step 4）。
##
## hakoniwa-mujoco-ship `include/ship/autonomy/cpa_ozt.hpp`（CpaOztCalculator）の GDScript 移植。
## 座標は ENU（x = 東, y = 北）[m]、速度 [m/s]、針路は真方位（0 = 北, 時計回り）。
## 倍精度のまま計算するため Vector2（単精度）は使わず、2 要素の Array [x, y] で受け渡す。
## tests/unit/ozt_parity で C++ 実装の出力（reference.json）と一致することを検証している。

const DEG_TO_RAD := PI / 180.0
## C++ と同じ 2° 刻みの全周スイープ
const COURSE_STEP_DEG := 2

## CpaOztCalculator::compute_cpa と同一。戻り値のキーは C++ の CpaMetrics のメンバ名。
static func compute_cpa(own_pos: Array, own_vel: Array, own_heading_rad: float,
		tgt_pos: Array, tgt_vel: Array,
		cpa_alarm_dist_m: float = 926.0, tcpa_alarm_time_s: float = 900.0) -> Dictionary:
	var px: float = tgt_pos[0] - own_pos[0]
	var py: float = tgt_pos[1] - own_pos[1]
	var vx: float = tgt_vel[0] - own_vel[0]
	var vy: float = tgt_vel[1] - own_vel[1]
	var res := {
		"cpa_distance_m": 0.0, "tcpa_seconds": 0.0, "bcr_distance_m": 0.0, "bct_seconds": 0.0,
		"bow_cross_ahead": false, "is_closing": false, "collision_risk": false,
	}

	var v_rel_sq := vx * vx + vy * vy
	if v_rel_sq < 1e-4:
		# 相対速度がほぼゼロ（同速・同コース）。C++ はここで BCR を計算せずに返す
		res["cpa_distance_m"] = sqrt(px * px + py * py)
		res["collision_risk"] = res["cpa_distance_m"] <= cpa_alarm_dist_m
		return res

	var tcpa := -(px * vx + py * vy) / v_rel_sq
	res["tcpa_seconds"] = tcpa
	res["is_closing"] = tcpa > 0.0
	var cx := px + vx * tcpa
	var cy := py + vy * tcpa
	res["cpa_distance_m"] = sqrt(cx * cx + cy * cy)
	res["collision_risk"] = res["is_closing"] and tcpa <= tcpa_alarm_time_s and res["cpa_distance_m"] <= cpa_alarm_dist_m

	# BCR: 他船の進路直線と自船首線の交点（Cramer の公式）
	var bx := sin(own_heading_rad)
	var by := cos(own_heading_rad)
	var tvx: float = tgt_vel[0]
	var tvy: float = tgt_vel[1]
	var det := bx * (-tvy) - by * (-tvx)
	if absf(det) > 1e-4:
		var s := (px * (-tvy) - py * (-tvx)) / det
		res["bcr_distance_m"] = s
		res["bct_seconds"] = (bx * py - by * px) / det
		res["bow_cross_ahead"] = s > 0.0
	else:
		res["bcr_distance_m"] = 1e6
		res["bct_seconds"] = 1e6
	return res

## CpaOztCalculator::compute_ozt と同一の結果（has_danger / min・max_danger_course_deg / danger_polygon）に加え、
## 危険進路の一覧 danger_courses_deg と各進路の衝突予測時刻 tcpa_s を返す。
## 注: C++ の min/max は危険進路列の先頭・末尾なので、危険域が真北（0°/360°）を跨ぐと 0〜358° になる。
## 描画には min/max ではなく danger_courses_deg（または danger_arcs()）を使うこと。
static func compute_ozt(own_pos: Array, own_speed_mps: float, tgt_pos: Array, tgt_vel: Array,
		safe_domain_m: float = 926.0, time_horizon_s: float = 900.0) -> Dictionary:
	var sector := {
		"has_danger": false, "min_danger_course_deg": 0.0, "max_danger_course_deg": 0.0,
		"danger_polygon": [], "danger_courses_deg": [], "tcpa_s": [],
	}
	if own_speed_mps <= 0.1:
		return sector
	for deg in range(0, 360, COURSE_STEP_DEG):
		var course := deg * DEG_TO_RAD
		var vel := [own_speed_mps * sin(course), own_speed_mps * cos(course)]
		var m := compute_cpa(own_pos, vel, course, tgt_pos, tgt_vel, safe_domain_m, time_horizon_s)
		if m["collision_risk"]:
			var t: float = m["tcpa_seconds"]
			sector["danger_courses_deg"].append(deg)
			sector["tcpa_s"].append(t)
			sector["danger_polygon"].append([own_pos[0] + vel[0] * t, own_pos[1] + vel[1] * t])
	var courses: Array = sector["danger_courses_deg"]
	if not courses.is_empty():
		sector["has_danger"] = true
		sector["min_danger_course_deg"] = float(courses.front())
		sector["max_danger_course_deg"] = float(courses.back())
	return sector

## 危険進路の一覧を連続区間 [start_deg, end_deg]（end は区間最後の進路、真北跨ぎは start > end）にまとめる。
static func danger_arcs(courses_deg: Array, step_deg: int = COURSE_STEP_DEG) -> Array:
	var arcs: Array = []
	if courses_deg.is_empty():
		return arcs
	if courses_deg.size() * step_deg >= 360:
		return [[0, 360 - step_deg]]
	for c in courses_deg:
		if not arcs.is_empty() and int(c) - int(arcs.back()[1]) == step_deg:
			arcs.back()[1] = int(c)
		else:
			arcs.append([int(c), int(c)])
	# 0° から始まる区間と 358° で終わる区間は真北を跨ぐ 1 区間
	if arcs.size() > 1 and arcs[0][0] == 0 and arcs.back()[1] == 360 - step_deg:
		arcs[0][0] = arcs.back()[0]
		arcs.pop_back()
	return arcs

## 真方位 course が危険進路一覧に含まれるか（最も近い step 刻みの進路で判定）
static func is_course_dangerous(courses_deg: Array, course_deg: float, step_deg: int = COURSE_STEP_DEG) -> bool:
	var bin := int(roundf(fposmod(course_deg, 360.0) / step_deg)) * step_deg % 360
	return courses_deg.has(bin)
