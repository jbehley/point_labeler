#version 330 core

layout (location = 0) in vec3  in_vertex;
layout (location = 1) in float in_remission;
layout (location = 2) in uint  in_label;
layout (location = 3) in uint  in_visible;

uniform mat4 mvp;

out vec4 color;
out uint out_label;

uniform int width;
uniform int height;
uniform vec2 window_pos;
uniform float radius;
uniform uint new_label;

void main()
{
  vec4 point = mvp * vec4(in_vertex, 1.0);
  gl_Position = mvp * vec4(in_vertex, 1.0);
  
  out_label = uint(30);
    
  vec3 pos = 0.5 * (point.xyz + vec3(1)) * (width, height, 1);
  pos.y = height - pos.y;
  
  if(pos.x >= 0 && pos.x < width  && pos.y >= 0 && pos.y < height && pos.z >= -1 && pos.z <= 1.0f)
  {
    float distance =  dot((pos.xy - window_pos), (pos.xy - window_pos));
    if( (distance < radius * radius) ) //(in_visible > uint(0)) &&
    {
      out_label = new_label;
    } 
  }
 

}
