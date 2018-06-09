#version 330 core

layout (location = 0) in vec3  in_vertex;
layout (location = 1) in float in_remission;
layout (location = 2) in uint  in_label;
layout (location = 3) in uint  in_visible;

uniform sampler2DRect label_colors;

#include "shaders/color.glsl"

// materials.
uniform mat4 mvp;

out vec4 color;

void main()
{
  vec4 in_color = texture(label_colors, vec2(in_label, 0));
  
  gl_Position = mvp * vec4(in_vertex, 1.0);
  if(in_visible == uint(0)) gl_Position = vec4(-10, -10, -10, 1);
  
  
  float r = in_remission * 0.7 + 0.3; // ensure r in [0.3, 1.0]
  vec3 hsv = rgb2hsv(in_color.rgb);
  hsv.b = max(hsv.b, 0.8);
  color = vec4(hsv2rgb(vec3(1, 1, r) * hsv), 1.0);
}
