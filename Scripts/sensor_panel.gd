extends PanelContainer
## センサ・通信の状態の画面（SHP-31、ADR D40）。画面の右に常時表示する。
##
## - センサ：1 行 1 センサ。物理の状態（生成側の真値。注入中の異常も）と、SUT が判断に使っているか・健全と判断したかを並べる
##   （物理は sensor_status ch 32、SUT は sut_sensor_use ch 33。並びは hakoniwa-mujoco-ship sensor_status.hpp）
## - 通信：船陸の回線（P2 でモデル化するまでは理想）と、SUT → FOC の自動化レベルの報告・FOC → SUT の指令
## - 自動化：AL・航行状態・指令針路・速力・ODD の余裕（autonomy_status ch 50）
## - 人間の担当：実船（MEGURI2040 の実証船 4 隻）でも自動化の範囲外のもの
## 表示の時刻は物理の時刻印（物理の行の stamp）。最後の更新から 3 周期＋2 s 来なければ「途絶」。

const MonitorSync = preload("res://Scripts/monitor_sync.gd")
const UPDATE_INTERVAL_S := 0.25

const SENSORS := [
	[MonitorSync.SENSOR_RADAR, "レーダ（X バンド）"],
	[MonitorSync.SENSOR_AIS, "AIS"],
	[MonitorSync.SENSOR_GNSS, "GNSS"],
	[MonitorSync.SENSOR_NAV, "航法（ジャイロ・ログ）"],
	[MonitorSync.SENSOR_CAMERA_VIS, "可視光カメラ"],
	[MonitorSync.SENSOR_CAMERA_IR, "赤外線カメラ"],
	[MonitorSync.SENSOR_LIDAR, "離着桟 LiDAR"],
	[MonitorSync.SENSOR_ECHO, "測深機"],
]
## 未接続のセンサをつなぐ予定の段（D40）
const PLANNED := {7: "P4", 8: "P4", 6: "P3", 5: "P3"}
const NAV_JA := {
	"NORMAL": "航路を追従", "AVOID_STBD": "右転して避航", "STAND_ON_ACT": "保持船の動作", "SLOW": "減速",
	"RETURN": "航路へ復帰", "MRS": "最小リスク状態", "REMOTE": "FOC が遠隔操船", "BERTH_APPROACH": "着桟の進入",
	"BERTH_CRAB": "横移動で着桟", "BERTHED": "接舷して保持",
}
const REASON_JA := {
	"NORMAL": "", "SENSOR": "センサ異常", "COMM_LATENCY": "通信の遅延", "COMM_LOST": "通信の途絶", "WEATHER": "気象海象",
	"CONGESTION": "輻輳", "CHANNEL": "指定航路", "ENGINE": "主機の負荷制限", "MRS": "最小リスク状態", "FOC": "FOC の介入",
	"RESTORED": "復帰の承認",
}
const AL_JA := {4: "AL4 自律", 3: "AL3 監視つき自律", 2: "AL2 縮退（最小リスク）", 1: "AL1 遠隔操船", 0: "AL0 手動"}
const C_OK := "7CFC9A"
const C_WARN := "FFD54A"
const C_BAD := "FF6B5E"
const C_OFF := "8A96A3"
const C_HEAD := "9CD3FF"

var monitor: Node = null

var _text: RichTextLabel
var _next_update := 0.0

func _ready() -> void:
	var font := SystemFont.new()
	font.font_names = PackedStringArray(["Yu Gothic UI", "Meiryo UI", "Meiryo", "MS Gothic", "Noto Sans CJK JP"])
	var theme_res := Theme.new()
	theme_res.default_font = font
	theme_res.default_font_size = 14
	theme = theme_res
	var style := StyleBoxFlat.new()
	style.bg_color = Color(0.03, 0.06, 0.1, 0.78)
	style.set_corner_radius_all(6)
	style.set_content_margin_all(10)
	add_theme_stylebox_override("panel", style)
	_text = RichTextLabel.new()
	_text.bbcode_enabled = true
	_text.fit_content = true
	_text.scroll_active = false
	_text.custom_minimum_size = Vector2(580, 0)
	add_child(_text)
	_text.text = "センサ・通信：受信待ち"

func _process(_delta: float) -> void:
	var now := Time.get_ticks_msec() / 1000.0
	if monitor == null or now < _next_update:
		return
	_next_update = now + UPDATE_INTERVAL_S
	_text.text = build_text()

## 表示の本文（BBCode）。検証のため外からも呼べる
func build_text() -> String:
	var t: float = monitor.plant_stamp if monitor.plant_stamp >= 0.0 else monitor.sim_time
	var lines := PackedStringArray()
	lines.append("[b][color=#%s]センサ（自船）[/color][/b]　[color=#%s]物理の真値 ｜ SUT の使い方[/color]" % [C_HEAD, C_OFF])
	for s in SENSORS:
		lines.append(_sensor_line(int(s[0]), str(s[1]), t))
	lines.append("")
	lines.append("[b][color=#%s]通信（船〜陸上支援センター）[/color][/b]" % C_HEAD)
	lines.append("  回線　[color=#%s]理想（遅延 0・途切れなし）[/color]　[color=#%s]Starlink・静止衛星・LTE のモデルは P2[/color]" % [C_WARN, C_OFF])
	# SUT は同じ内容でも 5 Hz で送るので、受信の新しさは SUT の行（sut_sensor_use）の時刻印で見る
	var sut_alive: bool = monitor.sut_stamp >= 0.0 and t - monitor.sut_stamp <= 2.0
	lines.append("  SUT → FOC 自動化レベルの報告　%s" % ("[color=#%s]受信中（5 Hz）[/color]" % C_OK if sut_alive and not monitor.autonomy.is_empty() else "[color=#%s]未受信[/color]" % C_OFF))
	var foc_text := "なし"
	if monitor.foc_count > 0:
		var f: Dictionary = monitor.foc
		foc_text = "%s %s（%s）" % [str(f.get("cmd", "?")), _course_speed(f), _age(monitor.foc_time)]
	lines.append("  FOC → SUT 指令　%s" % foc_text)
	lines.append("")
	lines.append("[b][color=#%s]自動化（SUT）[/color][/b]" % C_HEAD)
	if monitor.autonomy.is_empty():
		lines.append("  [color=#%s]未受信[/color]" % C_OFF)
	else:
		var a: Dictionary = monitor.autonomy
		var al := int(a.get("al", -1))
		var al_color := C_OK if al >= 3 else (C_WARN if al == 2 else C_BAD)
		var reason: String = REASON_JA.get(str(a.get("reason", "")), str(a.get("reason", "")))
		lines.append("  [color=#%s][b]%s[/b][/color]%s　ODD 余裕 %.2f" % [al_color, AL_JA.get(al, "AL%d" % al), ("（%s）" % reason) if not reason.is_empty() else "", float(a.get("odd", 0.0))])
		lines.append("  %s　指令 %03.0f°・%.1f kn" % [NAV_JA.get(str(a.get("nav", "")), str(a.get("nav", ""))), float(a.get("crs", 0.0)), float(a.get("spd", 0.0))])
	lines.append("")
	lines.append("[b][color=#%s]人間の担当（実船でも自動化の範囲外）[/color][/b]" % C_HEAD)
	lines.append("  [color=#%s]他船の灯火・形象物・発光信号、音響信号・無線（遭難信号を含む）[/color]" % C_OFF)
	return "\n".join(lines)

func _sensor_line(id: int, label: String, t: float) -> String:
	var p: Array = monitor.plant_rows.get(id, [])
	var u: Array = monitor.sut_rows.get(id, [])
	var state := int(p[0]) if not p.is_empty() else 0
	var dot: String
	var plant_text: String
	if p.is_empty() or state == 0:
		dot = "[color=#%s]○[/color]" % C_OFF
		plant_text = "[color=#%s]未接続%s[/color]" % [C_OFF, ("（%s で接続）" % PLANNED[id]) if PLANNED.has(id) else ""]
	elif _stale(p, t):
		dot = "[color=#%s]●[/color]" % C_BAD
		plant_text = "[color=#%s]途絶[/color]" % C_BAD
	elif state == 3:
		dot = "[color=#%s]●[/color]" % C_BAD
		plant_text = "[color=#%s]異常を注入中%s[/color]" % [C_BAD, _fault_text(id, p)]
	elif state == 2:
		dot = "[color=#%s]●[/color]" % C_WARN
		plant_text = "[color=#%s]劣化[/color]" % C_WARN
	else:
		dot = "[color=#%s]●[/color]" % C_OK
		plant_text = "正常"
	var value := _value_text(id, p, u)
	return "%s %s　%s ｜ %s%s" % [dot, label, plant_text, _use_text(id, u), ("　" + value) if not value.is_empty() else ""]

func _stale(p: Array, t: float) -> bool:
	var last := float(p[1])
	var rate := float(p[2])
	if rate <= 0.0:
		return false
	# AIS は報告の間隔が船の数・速力で変わる（Class A は 2〜10 s）ので最低 10 s 待つ
	return last < 0.0 or t - last > maxf(3.0 / rate + 2.0, 10.0)

static func _fault_text(id: int, p: Array) -> String:
	if id != MonitorSync.SENSOR_GNSS:
		return ""
	match int(p[3]):
		1:
			return "：なりすまし（偽の偏り %.0f m）" % float(p[4])
		2:
			return "：値の凍結"
		3:
			return "：途絶"
	return ""

func _use_text(id: int, u: Array) -> String:
	if u.is_empty() or int(u[0]) == 0:
		return "[color=#%s]SUT 未使用[/color]" % C_OFF
	var judge := int(u[6])
	if int(u[0]) == 1:
		if id == MonitorSync.SENSOR_GNSS and judge == 3:
			return "[color=#%s]SUT 不採用（推測航法で継続）[/color]" % C_BAD
		return "[color=#%s]SUT 監視のみ[/color]" % C_WARN
	if judge == 2:
		return "[color=#%s]SUT 使用中（疑い）[/color]" % C_WARN
	return "[color=#%s]SUT 使用中[/color]" % C_OK

func _value_text(id: int, p: Array, u: Array) -> String:
	match id:
		MonitorSync.SENSOR_RADAR:
			if p.is_empty():
				return ""
			var s := "検出 %d・クラッタ %d" % [int(p[3]), int(p[4])]
			if not u.is_empty():
				s += "・追尾 %d" % int(u[3])
			return s
		MonitorSync.SENSOR_AIS:
			if p.is_empty():
				return ""
			var s := "%d 文/分・AIS 船 %d" % [int(p[3]), int(p[4])]
			if not u.is_empty():
				s += "・受信 %d 隻" % int(u[3])
			return s
		MonitorSync.SENSOR_GNSS:
			if not u.is_empty() and int(u[0]) > 0:
				var bad := float(u[3]) > float(u[4])
				return "[color=#%s]推測航法との差 %.0f m（基準 %.0f m）[/color]" % [C_BAD if bad else "FFFFFF", float(u[3]), float(u[4])]
			return ""
		MonitorSync.SENSOR_NAV:
			return "%.1f kn・%03.0f°" % [float(p[3]), float(p[4])] if not p.is_empty() else ""
		MonitorSync.SENSOR_CAMERA_VIS, MonitorSync.SENSOR_CAMERA_IR:
			if p.is_empty() or float(p[3]) <= 0.0:
				return ""
			var vis := float(p[3])
			return "視程 %.1f km" % (vis / 1000.0) if vis < 9000.0 else ""
	return ""

static func _course_speed(j: Dictionary) -> String:
	if not j.has("crs"):
		return ""
	return "%03.0f°・%.1f kn" % [float(j.get("crs", 0.0)), float(j.get("spd", 0.0))]

func _msg_age(count: int, time_s: float) -> String:
	if count <= 0:
		return "[color=#%s]未受信[/color]" % C_OFF
	return "%d 件（最後 %s）" % [count, _age(time_s)]

func _age(time_s: float) -> String:
	var age: float = monitor.sim_time - time_s
	return "%.1f s 前" % maxf(age, 0.0)
