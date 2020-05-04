shader_type spatial;
// https://docs.godotengine.org/en/3.1/tutorials/shading/shading_reference/spatial_shader.html#render-modes
render_mode unshaded,cull_disabled,blend_mix,depth_draw_opaque;
uniform vec4 albedo : hint_color;
uniform sampler2D texture_albedo : hint_albedo;
uniform float specular;
uniform float metallic;
uniform float roughness : hint_range(0,1);
uniform float point_size : hint_range(0,128);
uniform vec3 uv1_scale;
uniform vec3 uv1_offset;
uniform vec3 uv2_scale;
uniform vec3 uv2_offset;
uniform float depth;
const float PI = 3.14159265359;

void vertex() {
	UV=UV*uv1_scale.xy+uv1_offset.xy;
  VERTEX.y = depth*cos(VERTEX.x*PI/2.0);
  VERTEX.x = 1.2*sin(VERTEX.x*PI/3.0);
}

void fragment() {
	vec2 base_uv = UV;
	vec4 albedo_tex = texture(texture_albedo,base_uv);
	ALBEDO = albedo.rgb * albedo_tex.rgb;
  ALPHA = albedo.a * albedo_tex.a;
	METALLIC = metallic;
	ROUGHNESS = roughness;
	SPECULAR = specular;
}
