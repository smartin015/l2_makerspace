shader_type spatial;
render_mode blend_mix,depth_draw_opaque,cull_back,diffuse_burley,specular_schlick_ggx,unshaded;
uniform float point_size : hint_range(0,128);
uniform sampler2D depth;

uniform float ppx;
uniform float ppy;
uniform float fx;
uniform float fy;
uniform vec4 coeffs;
const float FLT_EPSILON = 0.0000001;
vec3 rs2_deproject_pixel_to_point(float x, float y, float d) {
  // x = (x - ppx) / fx;
  // y = (y - ppy) / fy;
  
  // Assume RS2_DISTORTION_KANNALA_BRANDT4
  // See https://github.com/hiroMTB/ofxRealsense2/pull/4/files
  float rd = sqrt(x*x + y*y);
  if (rd < FLT_EPSILON) {
     rd = FLT_EPSILON;
  }
  float theta = rd;
  float theta2 = rd*rd;
  for (int i = 0; i < 4; i++) {
    float f = theta*(1.0 + theta2*(coeffs[0] + theta2*(coeffs[1] + theta2*(coeffs[2] + theta2*coeffs[3])))) - rd;
    if (abs(f) < FLT_EPSILON) {
      break;
    }
    float df = 1.0 + theta2*(3.0 * coeffs[0] + theta2*(5.0 * coeffs[1] + theta2*(7.0 * coeffs[2] + 9.0 * theta2*coeffs[3])));
    theta -= f / df;
    theta2 = theta*theta;
  }
  float r = tan(theta);
  x *= r / rd;
  y *= r / rd;
  return vec3(d * x, d, d * y);
}

void vertex() {
  POINT_SIZE = point_size;
  vec4 tex = texelFetch(depth, ivec2(int(UV.x * 256.0), int(UV.y * 256.0)), 0);
  // TODO apply rs2_deproject_pixel_to_point once intrinsics are known
  VERTEX.y = (
    tex.r * 256.0 + 
    tex.g);
}

void fragment() {
  vec3 tex = texture(depth,UV).rgb;
  float v = (tex.r * 256.0 + tex.g);
	ALBEDO = vec3(
    min(1.0, v/3.0), 
    min(1.0, (v-3.0)/3.0), 
    min(1.0, (v-6.0)/3.0));
}
