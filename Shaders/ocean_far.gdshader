// ocean_far.gdshader - 波面メッシュ外側の平面（潮位面）。波面メッシュと重なる内側は描画しない
shader_type spatial;
render_mode cull_back, diffuse_burley, specular_schlick_ggx;

uniform float hole_radius = 1150.0;
uniform vec3 color : source_color = vec3(0.02, 0.12, 0.22);

varying vec2 v_local;

void vertex() {
	v_local = VERTEX.xz;
}

void fragment() {
	if (length(v_local) < hole_radius) {
		discard;
	}
	ALBEDO = color;
	ROUGHNESS = 0.08;
	SPECULAR = 0.6;
}
