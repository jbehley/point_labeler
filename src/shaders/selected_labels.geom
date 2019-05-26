#version 330 core

layout(points) in;
layout(points, max_vertices = 1) out;

in POINT 
{
   uint label;
   bool valid;
} gs_in[];

out uint out_label;

void main()
{
       
    if(gs_in[0].valid)
	{
	   // peform here the filtering to only report points inside the polygon.
	   gl_Position = vec4(0,0,0,1);
	   out_label = gs_in[0].label;
	   EmitVertex();
	   EndPrimitive();
	}
}
