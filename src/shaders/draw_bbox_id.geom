#version 330 core

layout(points) in;
layout(triangle_strip, max_vertices = 18) out;


#include "shaders/color.glsl"

uniform mat4 mvp;

in BBOX 
{
  vec4 position;
  vec4 size;
  mat4 pose;
  uint id;
} gs_in[];

uniform bool use_custom_color;
uniform vec4 in_color;

out vec4 color;
flat out uint instance_id;
flat out uint label_id;


void main()
{
    vec4 p = gs_in[0].position;
    
    if(use_custom_color)
    {
      color = in_color;
    }
    else
    {
      vec3 hsv = vec3(fract(float(gs_in[0].id) / float(20)), 1.0, 1.0);
      color = vec4(hsv2rgb(hsv), 1.0);
    }
    
    instance_id = (gs_in[0].id >> 16);
    label_id = gs_in[0].id & uint(0xFFFF);
    
        
    mat4 pose = mat4(1.0f);
    

    
    vec4 size = gs_in[0].size;
    // ensure a minimum size of the drawn bounding boxes.
    size.x = max(size.x, 0.2);
    size.y = max(size.y, 0.2);
    size.z = max(size.z, 0.2);
    
    
    // 3 --- 2  7 --- 6
    // |     |  |     |
    // |     |  |     |
    // 4 --- 1  8 --- 5
    
    vec4 p1 = p + gs_in[0].pose * (vec4(0, 0, 0, 0) - 0.5f * size);
    vec4 p2 = p + gs_in[0].pose * (vec4(size.x, 0, 0, 0) - 0.5f * size);
    vec4 p3 = p + gs_in[0].pose * (vec4(size.x, size.y, 0, 0) - 0.5f * size);
    vec4 p4 = p + gs_in[0].pose * (vec4(0, size.y, 0, 0) - 0.5f * size);
    
    vec4 p5 = p + gs_in[0].pose * (vec4(0, 0, size.z, 0) - 0.5f * size);
    vec4 p6 = p + gs_in[0].pose * (vec4(size.x, 0, size.z, 0) - 0.5f * size);
    vec4 p7 = p + gs_in[0].pose * (vec4(size.x, size.y, size.z, 0) - 0.5f * size);
    vec4 p8 = p + gs_in[0].pose * (vec4(0, size.y, size.z, 0) - 0.5f * size);

    gl_Position = mvp * pose * p4; EmitVertex();
    gl_Position = mvp * pose * p3; EmitVertex();
    gl_Position = mvp * pose * p1; EmitVertex();
    gl_Position = mvp * pose * p2; EmitVertex();   
    gl_Position = mvp * pose * p5; EmitVertex();
    gl_Position = mvp * pose * p6; EmitVertex(); 
    gl_Position = mvp * pose * p8; EmitVertex();
    gl_Position = mvp * pose * p7; EmitVertex();
    gl_Position = mvp * pose * p4; EmitVertex();
    gl_Position = mvp * pose * p3; EmitVertex();
    EndPrimitive();

    gl_Position = mvp * pose * p7; EmitVertex();
    gl_Position = mvp * pose * p6; EmitVertex();
    gl_Position = mvp * pose * p3; EmitVertex();
    gl_Position = mvp * pose * p2; EmitVertex();
    EndPrimitive();
    
    gl_Position = mvp * pose * p5; EmitVertex();
    gl_Position = mvp * pose * p8; EmitVertex();
    gl_Position = mvp * pose * p1; EmitVertex();
    gl_Position = mvp * pose * p4; EmitVertex();
    EndPrimitive();
}