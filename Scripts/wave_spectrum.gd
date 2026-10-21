extends RefCounted
## JONSWAP / 多成分 Gerstner 波の決定論的波面生成（SHP-23 Step 3）。
##
## hakoniwa-mujoco-ship の include/ship/environment/wave_spectrum.hpp（および python/hakoniwa_wave）と
## 同一アルゴリズム・同一 PRNG（Xorshift32）の GDScript 移植。GDScript の float は倍精度なので
## 成分テーブルと CPU 評価は C++/Python と同精度で一致する（tests/unit/wave_parity で検証）。
## 座標は ENU（x=東, y=北, z=上）[m]、時刻 t [s] は物理と同じシミュレーション時刻。

const GRAVITY := 9.80665
const OMEGA_MIN := 0.20
const OMEGA_MAX := 3.20
const MAX_CHOPPINESS := 0.85

## WaveParams と同じキー: hs_m, tp_s, wave_dir_rad, gamma, seed, water_depth_m, tide_level_m, num_components
var params: Dictionary = {}
## 成分ごとの配列（WaveComponent の各フィールド）
var omega := PackedFloat64Array()
var k := PackedFloat64Array()
var kx := PackedFloat64Array()
var ky := PackedFloat64Array()
var amp := PackedFloat64Array()
var phase := PackedFloat64Array()
var dir_rad := PackedFloat64Array()
var choppiness := PackedFloat64Array()

var _rng_state: int = 1

static func default_params() -> Dictionary:
	return {
		"hs_m": 2.5, "tp_s": 8.0, "wave_dir_rad": 0.0, "gamma": 3.3, "seed": 42,
		"water_depth_m": 100.0, "tide_level_m": 0.0, "num_components": 32,
	}

func component_count() -> int:
	return omega.size()

func initialize(p: Dictionary) -> void:
	params = default_params()
	params.merge(p, true)
	var n := int(params["num_components"])
	var hs := float(params["hs_m"])
	var tp := float(params["tp_s"])
	var gamma := float(params["gamma"])
	var depth := float(params["water_depth_m"])
	var main_dir := float(params["wave_dir_rad"])
	# Packed*Array は値型のため個別に resize する
	omega.resize(n)
	k.resize(n)
	kx.resize(n)
	ky.resize(n)
	amp.resize(n)
	phase.resize(n)
	dir_rad.resize(n)
	choppiness.resize(n)

	_rng_state = int(params["seed"]) & 0xFFFFFFFF
	if _rng_state == 0:
		_rng_state = 1

	# 1. 周波数の離散化と JONSWAP 生スペクトル
	var d_omega := (OMEGA_MAX - OMEGA_MIN) / n
	var omega_p := (2.0 * PI) / maxf(1.0, tp)
	var target_m0 := pow(hs / 4.0, 2)
	var s_raw := PackedFloat64Array()
	s_raw.resize(n)
	var integral_s := 0.0
	for i in n:
		var w := OMEGA_MIN + (i + 0.5) * d_omega
		omega[i] = w
		var sigma := 0.07 if w <= omega_p else 0.09
		var r := exp(-pow(w - omega_p, 2) / (2.0 * pow(sigma * omega_p, 2)))
		var pm := pow(w, -5.0) * exp(-1.25 * pow(omega_p / w, 4.0))
		s_raw[i] = pm * pow(maxf(1.0, gamma), r)
		integral_s += s_raw[i] * d_omega

	# 2. Hs 正規化、分散関係、位相・方向（乱数の消費順は C++ と同じ: 位相 → 方向）
	var alpha_norm := (target_m0 / integral_s) if integral_s > 1e-9 else 1.0
	var total_ka := 0.0
	for i in n:
		amp[i] = sqrt(2.0 * (s_raw[i] * alpha_norm) * d_omega)
		k[i] = solve_dispersion_relation(omega[i], depth)
		phase[i] = _next_double() * 2.0 * PI
		dir_rad[i] = main_dir + (_next_double() - 0.5) * (PI / 2.0)
		kx[i] = k[i] * cos(dir_rad[i])
		ky[i] = k[i] * sin(dir_rad[i])
		total_ka += k[i] * amp[i]

	# 3. Gerstner チョッピーネス正規化
	for i in n:
		choppiness[i] = minf(MAX_CHOPPINESS, MAX_CHOPPINESS / total_ka) if total_ka > 1e-6 else 0.0

func get_elevation(x: float, y: float, t: float) -> float:
	var eta := float(params.get("tide_level_m", 0.0))
	for i in omega.size():
		eta += amp[i] * cos(kx[i] * x + ky[i] * y - omega[i] * t + phase[i])
	return eta

## WaveSurface 相当: elevation, slope_x, slope_y, normal([nx, ny, nz] ENU), disp_x, disp_y
func get_surface(x: float, y: float, t: float) -> Dictionary:
	var elevation := float(params.get("tide_level_m", 0.0))
	var slope_x := 0.0
	var slope_y := 0.0
	var disp_x := 0.0
	var disp_y := 0.0
	for i in omega.size():
		var a := kx[i] * x + ky[i] * y - omega[i] * t + phase[i]
		var c := cos(a)
		var s := sin(a)
		elevation += amp[i] * c
		slope_x -= amp[i] * kx[i] * s
		slope_y -= amp[i] * ky[i] * s
		if k[i] > 1e-9:
			disp_x -= choppiness[i] * amp[i] * (kx[i] / k[i]) * s
			disp_y -= choppiness[i] * amp[i] * (ky[i] / k[i]) * s
	var nx := -slope_x
	var ny := -slope_y
	var norm := sqrt(nx * nx + ny * ny + 1.0)
	return {
		"elevation": elevation, "slope_x": slope_x, "slope_y": slope_y,
		# Vector3 は単精度のため倍精度を保つ Array で返す（Python 版と同形式）
		"normal": [nx / norm, ny / norm, 1.0 / norm], "disp_x": disp_x, "disp_y": disp_y,
	}

## シェーダ uniform（Shaders/wave_common.gdshaderinc）用の成分テーブル: (kx, ky, amp, Q·amp/k)
func shader_k_amp() -> PackedVector4Array:
	var out := PackedVector4Array()
	out.resize(omega.size())
	for i in omega.size():
		var q := choppiness[i] * amp[i] / k[i] if k[i] > 1e-9 else 0.0
		out[i] = Vector4(kx[i], ky[i], amp[i], q)
	return out

## シェーダ uniform 用の位相。中心 (center_e, center_n) と時刻 t を倍精度で畳み込み 2π で剰余を取るため、
## シェーダ側は中心からの相対座標だけを単精度で扱えばよい（遠方・長時間でも精度が落ちない）。
func shader_phases(center_e: float, center_n: float, t: float) -> PackedFloat32Array:
	var out := PackedFloat32Array()
	out.resize(omega.size())
	for i in omega.size():
		out[i] = fposmod(phase[i] - omega[i] * t + kx[i] * center_e + ky[i] * center_n, TAU)
	return out

static func solve_dispersion_relation(w: float, depth_h: float) -> float:
	var k0 := (w * w) / GRAVITY
	if depth_h >= 200.0 or (k0 * depth_h) > 5.0:
		return k0
	var kk := k0
	for _iter in 10:
		var kh := kk * depth_h
		var tanh_kh := tanh(kh)
		var f := GRAVITY * kk * tanh_kh - (w * w)
		var df := GRAVITY * (tanh_kh + kh * (1.0 - tanh_kh * tanh_kh))
		var dk := f / df
		kk -= dk
		if absf(dk) < 1e-7:
			break
	return maxf(1e-4, kk)

## Xorshift32（GDScript の int は 64bit のため各段で 32bit にマスク）
func _next_double() -> float:
	var x := _rng_state
	x = (x ^ ((x << 13) & 0xFFFFFFFF)) & 0xFFFFFFFF
	x = (x ^ (x >> 17)) & 0xFFFFFFFF
	x = (x ^ ((x << 5) & 0xFFFFFFFF)) & 0xFFFFFFFF
	_rng_state = x
	return float(x) / 4294967296.0
