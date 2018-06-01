#version 330 core

layout (location = 0) in vec4  in_vertex;
layout (location = 1) in vec4  in_color;



// materials.
uniform mat4 projection;
uniform mat4 view;
uniform mat4 pose;

out vec4 color;

void main()
{
  gl_Position = projection * view * pose * in_vertex;
  color = in_color;
}