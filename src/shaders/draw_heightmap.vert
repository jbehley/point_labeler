#version 330 core

layout (location = 0) in vec2  in_coords;

uniform sampler2D heightmap;

uniform vec2 tilePos;
uniform float tileSize;
uniform float ground_resolution;
uniform float tileBoundary;

out TILE
{
  vec3 pos;
  float size;
} vs_out;

void main()
{
  ivec2 dim = textureSize(heightmap, 0);
  
  vec2 offset = 0.5 * dim * ground_resolution; 
  
  float height = texture(heightmap, in_coords).r;
  vec2 coords = vec2(in_coords.x * dim.x, in_coords.y * dim.y);
  vs_out.pos = vec3(tilePos - offset + ground_resolution * coords, height);
  vs_out.size  = ground_resolution;
}