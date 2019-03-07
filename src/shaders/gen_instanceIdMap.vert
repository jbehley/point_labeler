#version 330 core

layout (location = 0) in vec3 in_instanceId;

uniform float tex_width;
uniform float tex_height;

out float height;

void main()
{  
  gl_Position = vec4(2.0 * (in_instanceId.xy + vec2(0.5, 0.5)) / vec2(tex_width, tex_height) - 1.0, 0, 1);
  
  height = in_instanceId.z;
}