#version 330 core

layout (location = 0) in vec4 in_vertex;
layout (location = 1) in uint in_instance_label;

uniform mat4 pose;

uniform vec2 tilePos;
uniform float tileSize;
uniform float tileBoundary;

uniform uint label;

out float height;

void main()
{
  // lower 16 bits correspond to the label,
  // upper 16 bits correspond to the class.
  uint in_label = in_instance_label & uint(0xFFFF);
  uint in_instance = (in_instance_label >> 16) & uint(0xFFFF);
  
  float range = length(in_vertex.xyz);
  

  vec2 v = in_vertex.xy - tilePos;

  
  gl_Position = vec4(2.0 * v / (tileSize + tileBoundary), 0, 1);
  
  
  if(in_label != label || in_instance > uint(0)) gl_Position = vec4(-10, -10, -10, 1);  // ignore all other labels
  height = 1;
}