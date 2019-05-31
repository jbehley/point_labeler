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

//uniform bool planeRemoval;
//uniform int planeDimension;
//uniform float planeThreshold;
//uniform float planeDirection;

uniform bool planeRemovalNormal;
uniform vec3 planeNormal;
uniform float planeThresholdNormal;
uniform float planeDirectionNormal;
uniform mat4 plane_pose;

uniform bool removeLabel;


uniform bool labelInstances;
uniform int instanceLabelingMode;
uniform uint selectedInstanceId;
uniform uint selectedInstanceLabel;
uniform uint newInstanceId;
uniform bool hideLabeledInstances;


out uint out_label;

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
  // lower 16 bits correspond to the label,
  // upper 16 bits correspond to the class.
  uint label = in_label & uint(0xFFFF);
  uint instance = (in_label >> 16) & uint(0xFFFF);
  
  // NOTE: labeling points destroys the instance id.

  float range = length(in_vertex.xyz);
  float in_remission = in_vertex.w;
  
  vec4 point = mvp * vec4(in_vertex.xyz, 1.0);
  point = vec4(point.x/point.w, point.y/point.w, point.z/point.w, 1.0);
  gl_Position = mvp * vec4(in_vertex.xyz, 1.0);
  
  out_label = in_label;
    
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
  
  visible = visible && (!hideLabeledInstances || (instance == uint(0)));


  if(visible)
  {
    if(labelInstances)
    {
        // instanceLabelingMode: 
        //     0 -- add points in selected region from same class, 
        //     1 -- split points in selected region from same class, 
        //     2 -- remove points from given class and instance.
        //     3 -- create instance for given class.
        //     4 -- join instances.
    
        bool consistent_label = (label == selectedInstanceLabel);
        bool consistent_instanceId = ((instanceLabelingMode != 2 && instanceLabelingMode != 4)  || (instance == selectedInstanceId));
    
        if(consistent_label && consistent_instanceId)
        {
            if(instanceLabelingMode == 4)
            {
                out_label = (newInstanceId << 16) | label;
            }
            else
            {
		        for(int i = 0; i < numTriangles; ++i)
	            {
	              vec2 v1 = texture(triangles, vec2(3 * i + 0.5, 0.5)).xy * vec2(width, height);
	              vec2 v2 = texture(triangles, vec2(3 * i + 1.5, 0.5)).xy * vec2(width, height);
	              vec2 v3 = texture(triangles, vec2(3 * i + 2.5, 0.5)).xy * vec2(width, height);
	    
	              if(insideTriangle(pos.xy, v1, v2, v3))
	              {
	                if(instanceLabelingMode == 0) // add points
	                {
	                    out_label = (selectedInstanceId << 16) | label;
	                }
	                else if(instanceLabelingMode == 1) // split point
	                {
	                    out_label = (newInstanceId << 16) | label;
	                }
	                else if(instanceLabelingMode == 2) // remove points
	                {
	                   out_label = label;
	                }
	                else if(instanceLabelingMode == 3) // new instance
	                {
	                    out_label = (newInstanceId << 16) | label;
	                }
	                
	                break;
	              }
	            }
            }
        }
    }
    else
    {
	    if(pos.x >= 0 && pos.x < width  && pos.y >= 0 && pos.y < height && pos.z >= 0.0f && pos.z <= 1.0f)
	    {
	      if(labelingMode == 0)
	      {
	        float distance =  length(pos.xy - window_pos);
	        if(distance < radius) {
	          if(removeLabel && (overwrite || (label == new_label)))
	          {
	            out_label = uint(0);
	          }
	          else if(!removeLabel && (overwrite || (label == uint(0))))
	          {
	            out_label = new_label;
	          }
	        }
	      } 
	      else if(labelingMode == 1)
	      {
	        for(int i = 0; i < numTriangles; ++i)
	        {
	          vec2 v1 = texture(triangles, vec2(3 * i + 0.5, 0.5)).xy * vec2(width, height);
	          vec2 v2 = texture(triangles, vec2(3 * i + 1.5, 0.5)).xy * vec2(width, height);
	          vec2 v3 = texture(triangles, vec2(3 * i + 2.5, 0.5)).xy * vec2(width, height);
	
	          if(insideTriangle(pos.xy, v1, v2, v3) && (overwrite || label == uint(0)))
	          {
	            out_label = new_label;
	            break;
	          }
	        }
	      }
	    }
    }
  }
}
