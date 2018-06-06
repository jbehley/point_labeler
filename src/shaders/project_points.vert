#version 330 core

layout (location = 0) in vec3  in_vertex;
layout (location = 1) in float in_remission;
layout (location = 2) in vec4  in_color;
layout (location = 3) in uint  in_visible;


// materials.
uniform mat4 mvp;

out vec4 color;
out vec3 projected_point;

void main()
{
  
  gl_Position = mvp * vec4(in_vertex, 1.0);
  
  vec4 p = mvp * vec4(in_vertex, 1.0);
  projected_point = vec3(-10.f, -10.f, gl_VertexID);
  if(p.z > -1.0f || p.z < 1.0f) projected_point.xy = p.xy;
}
