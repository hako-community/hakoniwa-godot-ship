// wave_probe.gdshader - wave_common.gdshaderinc の GPU 評価値を画素へ書き出す検証用シェーダ
// 列 i = 検証点 i（probe_points[i] は中心からの ENU 相対座標）、行 0..3 = eta, slope_E, slope_N, 既知定数
// 値は 24bit 固定小数（RGB）へ符号化する。
shader_type canvas_item;

#include "res://Shaders/wave_common.gdshaderinc"

uniform int probe_count = 64;
uniform vec2 probe_points[64];
uniform float eta_range = 64.0;
uniform float slope_range = 4.0;
uniform float sanity_value = 0.123456;

vec4 encode24(float v, float range) {
	float u = floor(clamp(v / range + 0.5, 0.0, 1.0) * 16777215.0 + 0.5);
	float r = floor(u / 65536.0);
	float g = floor((u - r * 65536.0) / 256.0);
	float b = u - r * 65536.0 - g * 256.0;
	return vec4(r, g, b, 255.0) / 255.0;
}

void fragment() {
	int i = int(floor(UV.x * float(probe_count)));
	int row = int(floor(UV.y * 4.0));
	float eta;
	vec2 slope;
	vec2 disp;
	wave_eval(probe_points[i], eta, slope, disp);
	if (row == 0) {
		COLOR = encode24(eta, eta_range);
	} else if (row == 1) {
		COLOR = encode24(slope.x, slope_range);
	} else if (row == 2) {
		COLOR = encode24(slope.y, slope_range);
	} else {
		COLOR = encode24(sanity_value, slope_range);
	}
}
