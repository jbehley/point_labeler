#version 330 core

layout (location = 0) in vec3  in_vertex;
layout (location = 1) in float in_remission;
layout (location = 2) in uint  in_label;
layout (location = 3) in uint  in_visible;

uniform mat4 mvp;
uniform int width;
uniform int height;
uniform vec2 window_pos;
uniform float radius;
uniform uint new_label;
uniform bool overwrite;

out uint out_label;


void main()
{
  vec4 point = mvp * vec4(in_vertex, 1.0);
  point = vec4(point.x/point.w, point.y/point.w, point.z/point.w, 1.0);
  gl_Position = mvp * vec4(in_vertex, 1.0);
  
  out_label = in_label;
    
  vec3 pos =  vec3(0.5f * (point.x + 1.0) * width, 0.5f * (point.y + 1.0) * height, 0.5f * (point.z + 1.0)); 
  pos.y = height - pos.y;
  
  if(pos.x >= 0 && pos.x < width  && pos.y >= 0 && pos.y < height && pos.z >= 0.0f && pos.z <= 1.0f)
  {

    float distance =  length(pos.xy - window_pos);
    if((in_visible > uint(0)) && (distance < radius) && (overwrite || in_label == uint(0)))
    {
      out_label = new_label;
    } 
  }
 

}
