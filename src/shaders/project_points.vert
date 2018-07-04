#version 330 core

layout (location = 0) in vec4  in_vertex;
layout (location = 1) in vec4  in_color;
layout (location = 2) in uint  in_visible;


// materials.
uniform mat4 mvp;


out vec3 projected_point;

void main()
{
  
  gl_Position = mvp * vec4(in_vertex.xyz, 1.0);
  
  vec4 p = mvp * vec4(in_vertex.xyz, 1.0);
  projected_point = vec3(-10.f, -10.f, gl_VertexID);
  // projected_point = p.xyz;
  if(p.z > -1.0f || p.z < 1.0f) projected_point.xy = p.xy;
}
