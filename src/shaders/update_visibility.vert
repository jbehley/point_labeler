#version 330 core

layout (location = 0) in vec4  in_vertex;
layout (location = 1) in uint  in_label;
layout (location = 2) in uint  in_visible;

uniform uint label;
uniform uint visibility;

out uint out_visible;

void main()
{
  out_visible = in_visible;
  if(in_label == label) out_visible = visibility;
}