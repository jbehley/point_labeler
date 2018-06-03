#version 330 core

layout (location = 0) in vec4  in_vertex;
layout (location = 1) in float in_remission;
layout (location = 2) in vec4  in_color;
layout (location = 3) in uint  in_visible;


#include "shaders/color.glsl"

// materials.
uniform mat4 mvp;

out vec4 color;

void main()
{
  
  gl_Position = mvp * in_vertex;
  
  float r = max(in_remission,0.3);
  color = vec4(hsv2rgb(vec3(1, 1, r) * rgb2hsv(in_color.rgb)), 1.0);
}
