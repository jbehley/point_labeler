#version 330 core

flat in uint instance_id;
flat in uint label_id;

layout (location = 0) out vec2 out_color;

void main()
{
  out_color = vec2(instance_id, label_id);
}