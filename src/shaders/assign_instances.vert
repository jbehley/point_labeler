#version 330 core

layout (location = 0) in vec4 in_vertex;
layout (location = 1) in uint in_instance_label;

uniform sampler2D instanceIdMap;
uniform vec2  tilePos;
uniform float tileSize;
uniform uint  label;
uniform float tileBoundary;

out uint out_label;


void main()
{
  // lower 16 bits correspond to the label,
  // upper 16 bits correspond to the class.
  uint in_label = in_instance_label & uint(0xFFFF);
  uint instance_id = (in_instance_label >> 16) & uint(0xFFFF);
  
  vec2 v = in_vertex.xy - tilePos;

  uint id = uint(texture(instanceIdMap, v / (tileSize + 2.0 * tileBoundary) + 0.5).r);
  
  out_label = in_instance_label;
  if(in_label == label && id > uint(0)) out_label = (id << 16)  + in_label;
}
