// wave_common.gdshaderinc - 決定論的 JONSWAP / Gerstner 波面の GPU 評価（SHP-23 Step 3）
//
// wave_spectrum.hpp の get_surface() と同じ式。成分テーブルと位相は CPU（Scripts/wave_spectrum.gd）が
// shader_k_amp() / shader_phases() で倍精度計算して渡す。位相には評価中心と時刻が畳み込まれているので、
// 引数 d は「評価中心からの ENU 相対座標 (東, 北) [m]」。

#define WAVE_MAX_COMPONENTS 64

uniform int wave_count = 0;
uniform vec4 wave_k_amp[WAVE_MAX_COMPONENTS];   // (kx, ky, amp, choppiness * amp / k)
uniform float wave_phase[WAVE_MAX_COMPONENTS];  // phase - omega*t + kx*cE + ky*cN  (mod 2π)
uniform float wave_tide = 0.0;

// eta: 標高 [m]、slope: (d eta/dE, d eta/dN)、disp: Gerstner 水平変位 (dE, dN) [m]
void wave_eval(vec2 d, out float eta, out vec2 slope, out vec2 disp) {
	eta = wave_tide;
	slope = vec2(0.0);
	disp = vec2(0.0);
	for (int i = 0; i < wave_count; i++) {
		vec4 c = wave_k_amp[i];
		float a = c.x * d.x + c.y * d.y + wave_phase[i];
		float ca = cos(a);
		float sa = sin(a);
		eta += c.z * ca;
		slope -= c.z * c.xy * sa;
		disp -= c.w * c.xy * sa;
	}
}
