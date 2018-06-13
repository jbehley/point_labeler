#version 330 core

layout (location = 0) in vec2  in_vertex;

uniform sampler2DRect label_colors;

uniform int width;
uniform int height;

uniform uint label;

out vec4 color;

void main()
{

  gl_Position = vec4(2.0 * in_vertex.x/width - 1.0, 2.0 * (1.0-in_vertex.y/height) - 1.0,0.0, 1.0);

  color = texture(label_colors, vec2(label, 0));
}