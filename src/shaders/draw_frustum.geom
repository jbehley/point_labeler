#version 330 core

layout(points) in;
layout(line_strip, max_vertices = 13) out;

uniform mat4 mvp;
uniform mat4 pose;
uniform float size;

uniform int width;
uniform int height;


out vec4 color;

void main()
{
    color = vec4(0.5, 0.8, 0.7, 1);
    
    float ratio = float(height)/width;
    
    gl_Position = mvp * pose * vec4(0.3, -0.5, ratio * 0.5, 1);
    EmitVertex();
    gl_Position = mvp * pose * vec4(0.3, 0.5, ratio * 0.5, 1);
    EmitVertex();
    gl_Position = mvp * pose * vec4(0.3, 0.5, -ratio * 0.5, 1);
    EmitVertex();
    gl_Position = mvp * pose * vec4(0.3, -0.5, -ratio * 0.5, 1);
    EmitVertex();
    gl_Position = mvp * pose * vec4(0.3, -0.5, ratio * 0.5, 1);
    EmitVertex();
    EndPrimitive();
    
    
    gl_Position = mvp * pose * vec4(0.3, -0.5, ratio * 0.5, 1);
    EmitVertex();
    gl_Position = mvp * pose * vec4(0.0, 0, 0, 1);
    EmitVertex();
    EndPrimitive();
    
    
    gl_Position = mvp * pose * vec4(0.3, 0.5, ratio * 0.5, 1);
    EmitVertex();
    gl_Position = mvp * pose * vec4(0.0, 0, 0, 1);
    EmitVertex();
    EndPrimitive();

    gl_Position = mvp * pose * vec4(0.3, 0.5, -ratio * 0.5, 1);
    EmitVertex();
    gl_Position = mvp * pose * vec4(0.0, 0, 0, 1);
    EmitVertex();
    EndPrimitive();
    
    gl_Position = mvp * pose * vec4(0.3, -0.5, -ratio * 0.5, 1);
    EmitVertex();
    gl_Position = mvp * pose * vec4(0.0, 0, 0, 1);
    EmitVertex();
    EndPrimitive();
}
