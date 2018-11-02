#version 330 core

layout (location = 0) in vec4  in_vertex;
layout (location = 1) in uint  in_instance_label;
layout (location = 2) in uint  in_visible;

uniform uint label;
uniform uint visibility;

out uint out_visible;

void main()
{
  // lower 16 bits correspond to the label,
  // upper 16 bits correspond to the class.
  uint in_label = in_instance_label & uint(0xFFFF);
  uint instance = (in_instance_label >> 16) & uint(0xFFFF);

  out_visible = in_visible;
  if(in_label == label) out_visible = visibility;
}