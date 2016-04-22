uniform mat4 transform;

attribute vec4 vertex;
attribute vec4 color;

varying vec4 vertColor;

void main() {

  vec4 myVector = vec4(vertex[0]*-1000, vertex[1]*1000, vertex[2]*-1000,1.0);
  gl_Position = transform * myVector;
  if(myVector[2]>-4000 && color[2]-color[1]>10 && color[2]-color[0]>10){vertColor = vec4(0.0,256.0,0.0,256.0);}
  //else if(myVector[2]>-4000 && color[1]-color[0]>20 && color[1]-color[2]>20){vertColor = vec4(0.0,256.0,0.0,256.0);}
  //else if(myVector[2]>-4000 && color[0]-color[1]>20 && color[0]-color[2]>20){vertColor = vec4(256.0,0.0,0.0,256.0);}
  else{vertColor = vec4(0.0,0.0,0.0,0.0);}   
  //vertColor = color;
}

//color notation:bgr