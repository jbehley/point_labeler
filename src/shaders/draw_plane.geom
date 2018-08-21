#version 330 core

layout(points) in;
layout(triangle_strip, max_vertices = 6) out;

uniform mat4 mvp;

uniform bool removeGround;
uniform float groundThreshold;

uniform bool planeRemoval;
uniform int planeDimension;
uniform float planeThreshold;
uniform float planeDirection;

uniform bool planeRemovalNormal;
uniform vec3 planeNormal;
uniform float planeThresholdNormal;
uniform float planeDirectionNormal;
uniform bool carAsBase;

uniform mat4 pose;

uniform vec2 tilePos;
uniform float tileSize;

out vec4 color;

void main() {
  
    vec4 n = pose * vec4(planeDirection * float(planeDimension == 0), planeDirection * float(planeDimension == 1), planeDirection * float(planeDimension == 2), 0);
    vec4 p = vec4(planeThreshold * n.xyz, 1.0);
        
    if(planeRemovalNormal){
      vec3 pn = planeNormal;
      if(carAsBase) pn = (pose * vec4(planeNormal, 0.0)).xyz;
      
      n = vec4(pn, 0.0);
      
      p = vec4(planeThresholdNormal * n.xyz, 1.0);
      p.x = p.x + tilePos.x;
      p.y = p.y + tilePos.y;
    }
    
    vec4 u = normalize(vec4(n.y - n.z, -n.x, n.x, 0.0f));
    vec4 v = vec4(normalize(cross(n.xyz, u.xyz)), 0.0f); 
  


    color = vec4(0, 0, 1, 0.5);
    
    float radius = 10.0;
    
    vec4 ru = radius * u;
    vec4 rv = radius * v;
        
    gl_Position = mvp * (p - ru - rv);
    EmitVertex();
    
    gl_Position =  mvp * (p + ru - rv);
    EmitVertex();
    
    gl_Position =  mvp * (p - ru + rv);
    EmitVertex();
    
    gl_Position =  mvp * (p + ru + rv);
    EmitVertex();
    EndPrimitive();
    
}
