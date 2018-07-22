#version 330 core

layout(points) in;
layout(triangle_strip, max_vertices = 6) out;

in TILE 
{
  vec3 pos;
  float size;
} gs_in[];

uniform mat4 mvp;

out vec4 color;

void main() {
  
    vec3  p = gs_in[0].pos;
    float size = gs_in[0].size - 0.01;
    float height = p.z;
    
    color = vec4(0.4, 0.4, 0.4, 0.5);
    
    gl_Position = mvp * vec4(p.x - 0.5*size, p.y - 0.5*size, height, 1);
    EmitVertex();
    gl_Position = mvp * vec4(p.x + 0.5*size, p.y - 0.5*size, height, 1);
    EmitVertex();
    gl_Position = mvp * vec4(p.x - 0.5*size, p.y + 0.5*size, height, 1);
    EmitVertex();
    EndPrimitive();
    
    gl_Position = mvp * vec4(p.x - 0.5*size, p.y + 0.5*size, height, 1);
    EmitVertex();
    gl_Position = mvp * vec4(p.x + 0.5*size, p.y - 0.5*size, height, 1);
    EmitVertex();
    gl_Position = mvp * vec4(p.x + 0.5*size, p.y + 0.5*size, height, 1);
    EmitVertex();
    EndPrimitive();
    
}