#version 330 core

layout (location = 0) in vec3  in_vertex;
layout (location = 1) in float in_remission;
layout (location = 2) in uint  in_label;
layout (location = 3) in uint  in_visible;

uniform uint scan;

uniform mat4 pose;

uniform float minRange;
uniform float maxRange;

uniform vec2 tilePos;
uniform float tileSize;
uniform float tileBoundary;

uniform bool addCarPoints;

out POINT
{
  bool valid;
  vec4 point;
  uint label;
  uint visible;
  vec2 scanindex;
} vs_out;


void main()
{
  float range = length(in_vertex);
  
  vec4 v_global = pose * vec4(in_vertex, 1.0);
  vec2 v = v_global.xy - tilePos;
  
  bool inside_tile = (abs(v.x) < 0.5 * (tileSize + tileBoundary) && abs(v.y) < 0.5 * (tileSize + tileBoundary));
  bool out_of_range = range < minRange || range > maxRange;
  bool is_car_point = (in_vertex.x < 3.0 && in_vertex.x > -2.0 && abs(in_vertex.y) < 2.0);
   
  vs_out.valid = false;
  
  if(inside_tile && !out_of_range && (addCarPoints || !is_car_point ))
  {
    vs_out.valid = true;
    vs_out.point = vec4(v_global.xyz, in_remission);
    vs_out.label = in_label;
    vs_out.visible = in_visible;
    vs_out.scanindex = vec2(scan, gl_VertexID);
  }
}