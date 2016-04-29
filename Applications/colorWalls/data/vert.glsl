uniform mat4 transform;

attribute vec4 vertex;
attribute vec4 color;
attribute vec4 my;

varying vec4 vertColor;

void main() {

  vec4 myVector = vec4(vertex[0]*-1000, vertex[1]*1000, vertex[2]*-1000,1.0);
  gl_Position = transform * myVector;

  float r = float(color[2])*1.0;
  float g = float(color[1])*1.0;
  float b = float(color[0])*1.0;
  
  if(myVector[2]>-2500){

		
	if( r-b>35 && r-g>35){
		vertColor = vec4(200.0,200.0,256.0,256.0);
	}
	/*else if (g-b>15 && g-r>15){
		vertColor = vec4(100.0,256.0,256.0,256.0);
	}*/					
	else{vertColor = vec4(0.0,0.0,0.0,0.0);}
    }
  else{vertColor = vec4(0.0,0.0,0.0,0.0);}   
  //vertColor = color;
  
}

//color notation:bgr