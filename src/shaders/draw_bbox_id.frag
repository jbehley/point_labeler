#version 330 core

in float instance_id;

layout (location = 0) out float out_color;

void main()
{
  out_color = instance_id;
}