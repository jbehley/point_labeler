#version 330 core

layout (location = 0) in vec3  in_vertex;

// uniform mat4 pose;

uniform vec2 tilePos;
uniform float tileSize;

uniform float minHeight;
uniform float maxHeight;

out float height;

void main()
{
  float range = length(in_vertex);
  
  // vec4 v_global = vec4(in_vertex, 1.0);
  vec2 v = vec4(in_vertex, 1.0).xy - tilePos;
  // bool visible =  (abs(v.x) < 0.5 * tileSize && abs(v.y) < 0.5 * tileSize);
  
  gl_Position = vec4(2.0 * v/tileSize, (in_vertex.z-minHeight) / (maxHeight-minHeight), 1);
  
  //if(!visible) gl_Position = vec4(-10, -10, -10, 1);
  height = in_vertex.z;
}