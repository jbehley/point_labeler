#version 330

layout (location = 0) out float out_height; 
in float height;

uniform sampler2D heightmap;

in vec2 texCoords;

void main()
{
  ivec2 dim = textureSize(heightmap, 0);

  out_height = texture(heightmap, texCoords).r;
  int count = 1;
 
 
  
  for(int i = -1; i <= 1; i += 2)
  {
    for(int  j = -1; j <= 1; j += 2)
    {
      vec2 c = vec2(texCoords.x + i / dim.x, texCoords.y + j / dim.y);
      if(c.x < 0.0 || c.x > 1.0 || c.y < 0.0 || c.y > 1.0) continue;
      out_height += texture(heightmap, c).r;
      count += 1;
    } 
  }
  
  out_height = 1.0/float(count) * out_height;
}