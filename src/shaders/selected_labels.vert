#version 330 core

layout (location = 0) in vec4  in_vertex;
layout (location = 1) in uint  in_label;
layout (location = 2) in uint  in_visible;

uniform mat4 mvp;

uniform int width;
uniform int height;
uniform vec2 window_pos;
uniform float radius;
uniform uint new_label;
uniform bool overwrite;

uniform int labelingMode;
uniform sampler2DRect triangles;
uniform int numTriangles;

uniform bool removeGround;
uniform float groundThreshold;

uniform sampler2D heightMap;

uniform vec2 tilePos;
uniform float tileSize;

uniform bool planeRemovalNormal;
uniform vec3 planeNormal;
uniform float planeThresholdNormal;
uniform float planeDirectionNormal;
uniform mat4 plane_pose;

uniform bool hideLabeledInstances;


out POINT
{
   uint label;
   bool valid;
} vs_out;

bool insideTriangle(vec2 p, vec2 v1, vec2 v2, vec2 v3) {
  float b0 = ((v2.x - v1.x) * (v3.y - v1.y) - (v3.x - v1.x) * (v2.y - v1.y));

  if (abs(b0) > 0) {
    // compute barycentric coordinates.
    float b1 = (((v2.x - p.x) * (v3.y - p.y) - (v3.x - p.x) * (v2.y - p.y)) / b0);
    float b2 = (((v3.x - p.x) * (v1.y - p.y) - (v1.x - p.x) * (v3.y - p.y)) / b0);
    float b3 = 1.0f - b1 - b2;

    // only if all are greater equal 0, the point can be inside.
    return (b1 > 0) && (b2 > 0) && (b3 > 0);
  }

  return false;
}

void main()
{
  float range = length(in_vertex.xyz);
  float in_remission = in_vertex.w;
  
  vec4 point = mvp * vec4(in_vertex.xyz, 1.0);
  point = vec4(point.x/point.w, point.y/point.w, point.z/point.w, 1.0);

  vs_out.valid = false;
    
  vec3 pos =  vec3(0.5f * (point.x + 1.0) * width, 0.5f * (point.y + 1.0) * height, 0.5f * (point.z + 1.0)); 
  pos.y = height - pos.y;
  
  vec4 v_global = vec4(in_vertex.xyz, 1.0);
  vec2 v = v_global.xy - tilePos;
  
  //vec4 plane_normal = pose * vec4(planeDirection * float(planeDimension == 0), planeDirection * float(planeDimension == 1), planeDirection * float(planeDimension == 2), 0);
  
  bool visible = (in_visible > uint(0)) && (!removeGround || v_global.z > texture(heightMap, v / tileSize + 0.5).r + groundThreshold); 

  //if(planeRemoval) visible = visible && ((dot(plane_normal, in_vertex) - planeThreshold)  < 0);
  
  if(planeRemovalNormal){
    vec3 pn = (plane_pose * vec4(planeNormal, 0.0)).xyz;
    vec3 po = (plane_pose * vec4(0,0,0,1)).xyz;
    
    float scalar_product = dot(in_vertex.xyz - po.xyz, pn);
    
    visible = visible && (planeDirectionNormal * (scalar_product - planeThresholdNormal) < 0);
  }
  
  uint instance = (in_label >> 16) & uint(0xFFFF);
  visible = visible && (!hideLabeledInstances || (instance == uint(0)));


  if(visible)
  {
    for(int i = 0; i < numTriangles; ++i)
    {
      vec2 v1 = texture(triangles, vec2(3 * i + 0.5, 0.5)).xy * vec2(width, height);
      vec2 v2 = texture(triangles, vec2(3 * i + 1.5, 0.5)).xy * vec2(width, height);
      vec2 v3 = texture(triangles, vec2(3 * i + 2.5, 0.5)).xy * vec2(width, height);

      if(insideTriangle(pos.xy, v1, v2, v3))
      {
        vs_out.valid = true; // report label.
        vs_out.label = in_label;
        
        break;
      }
    }
  }
}
