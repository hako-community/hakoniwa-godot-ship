// ocean_gerstner.gdshader - 物理と同一の決定論的波面を描画する海面（SHP-23 Step 3）
//
// メッシュ（Scripts/ocean.gd）は評価中心に置かれ、回転なし。VERTEX.xz は中心からの Godot 相対座標で、
// ENU へは 東 = -z, 北 = -x（FLU→Godot: x→-Z, y→-X）。
// gerstner_displacement = 0（既定）では高さ場のみで、物理の浮力計算 eta(x, y, t) と同じ点で同じ標高になる。
// 1 にすると水平変位で波頭が尖るが、頂点が水平移動するため物理の標高とは一致しなくなる（見た目用）。
shader_type spatial;
render_mode cull_back, diffuse_burley, specular_schlick_ggx;

#include "res://Shaders/wave_common.gdshaderinc"

uniform float gerstner_displacement : hint_range(0.0, 1.0) = 0.0;
// 外周で振幅を 0 へ落とし、周囲の平面（潮位面）へ継ぎ目なく接続する。検証範囲は fade_start の内側
uniform float fade_start = 950.0;
uniform float fade_end = 1180.0;
uniform vec3 deep_color : source_color = vec3(0.02, 0.12, 0.22);
uniform vec3 shallow_color : source_color = vec3(0.10, 0.36, 0.48);
uniform float crest_highlight = 0.35;

varying float v_height;

void vertex() {
	vec2 d = vec2(-VERTEX.z, -VERTEX.x);
	float eta;
	vec2 slope;
	vec2 disp;
	wave_eval(d, eta, slope, disp);

	float fade = 1.0 - smoothstep(fade_start, fade_end, length(VERTEX.xz));
	eta = wave_tide + (eta - wave_tide) * fade;
	slope *= fade;
	disp *= fade * gerstner_displacement;

	VERTEX.y = eta;
	VERTEX.x += -disp.y;
	VERTEX.z += -disp.x;
	// ENU 法線 (-sE, -sN, 1) → Godot (-nN, nUp, -nE) = (sN, 1, sE)
	NORMAL = normalize(vec3(slope.y, 1.0, slope.x));
	v_height = eta - wave_tide;
}

void fragment() {
	float fresnel = pow(1.0 - clamp(dot(NORMAL, VIEW), 0.0, 1.0), 4.0);
	float crest = clamp(v_height * crest_highlight, 0.0, 1.0);
	ALBEDO = mix(deep_color, shallow_color, clamp(fresnel + crest * 0.5, 0.0, 1.0));
	ROUGHNESS = 0.08;
	METALLIC = 0.0;
	SPECULAR = 0.6;
}
