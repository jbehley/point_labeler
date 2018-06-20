#version 330 core

layout (location = 0) out float out_height; 
in float height;

void main()
{
  out_height = height;
}