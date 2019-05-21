#version 330 core

layout (location = 0) in vec4  in_position_yaw;
layout (location = 1) in vec4  in_size_id;
layout (location = 2) in uint  in_instance_label;

out BBOX 
{
  vec4 position;
  vec4 size;
  mat4 pose;
  uint id;
} vs_out;


void main()
{
  vs_out.position = vec4(in_position_yaw.xyz, 1.0);
  vs_out.size = vec4(in_size_id.xyz, 0.0);
  float yaw = in_position_yaw.w;
  vs_out.pose = mat4(cos(yaw), sin(yaw), 0, 0,  // first column
                     -sin(yaw), cos(yaw), 0, 0, // second column
                     0, 0, 1, 0,                // third column
                     0, 0, 0, 1);               // fourth column
  vs_out.id = in_instance_label;
}