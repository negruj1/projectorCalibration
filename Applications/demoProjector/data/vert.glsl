uniform mat4 transform;

attribute vec4 vertex;
attribute vec4 color;

varying vec4 vertColor;

void main() {

  vec4 myVector = vec4(vertex[0]*-1000, vertex[1]*1000, vertex[2]*-1000,1.0);
  gl_Position = transform * myVector;

  float r = float(color[2])*1.0;
  float g = float(color[1])*1.0;
  float b = float(color[0])*1.0;
  
  float intensity = (r+g+b)/3.0;
  float hue = 57.0*cos(     (2*r-g-b)/  (   2*sqrt((r-g)*(r-g)+(r-g)*(g-b))   )     );
  float saturation = 1-min(min(r,g),min(g,b))/intensity;

  if(myVector[2]>-1700 && intensity>40.0){

	if(myVector[0]>270){//rechts

		if(myVector[1]>0){
			if( g-b>16){
				vertColor = vec4(100.0,256.0,100.0,256.0);
			}
			else{
				vertColor = vec4(0.0,0.0,256.0,256.0);
			}
		}
		else{
			if( g-b>20 && g-r>8){
				vertColor = vec4(100.0,256.0,100.0,256.0);
			}
			else{
				vertColor = vec4(256.0,100.0,100.0,256.0);
			}
		}

			
	}
	else{//links
		if(  g-b>10){
			vertColor = vec4(100.0,256.0,100.0,256.0);
		}
		else{
			vertColor = vec4(256.0,256.0,0.0,256.0);
		}	
		
	} 
	
    }
  else{vertColor = vec4(0.0,0.0,0.0,0.0);}   
  //vertColor = color;
  
}

//color notation:bgr