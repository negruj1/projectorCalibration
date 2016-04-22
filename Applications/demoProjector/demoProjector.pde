import java.nio.*;
import KinectPV2.*;
import com.jogamp.opengl.GL2;

import KinectPV2.*;

private KinectPV2 kinect;

PGraphicsOpenGL pg; 
GL2 gl;

//values for the 3d scene
//rotation
float a = 3.14;
//z scale value
int   zval     = -200;
//value to scale the point cloud
float scaleVal = 30;

int maxD = 4000; // 4m
int minD = 0;  //  0m

float resolutionX = 1280;
float resolutionY = 800;

//rotation
float rotY = 0;
float rotZ = 0;
float rotX = PI;

float depthVal = 0;

int vertLoc;
int colorLoc;

//openGL instances
//render ot openGL object
PGL pgl;

//create a shader
PShader sh;


//VBO buffer location in the GPU
int vertexVboId;
int colorVboId;

public void setup() {
  fullScreen(P3D);

  kinect = new KinectPV2(this);

  kinect.enableDepthImg(true);
  kinect.enableColorImg(true);
  kinect.enableColorPointCloud(true);

  kinect.init();

  //create shader object with a vertex shader and a fragment shader
  sh = loadShader("frag.glsl", "vert.glsl");


  //create VBO

  PGL pgl = beginPGL();

  // allocate buffer big enough to get all VBO ids back
  IntBuffer intBuffer = IntBuffer.allocate(2);
  pgl.genBuffers(2, intBuffer);

  //memory location of the VBO
  vertexVboId = intBuffer.get(0);
  colorVboId = intBuffer.get(1);
  
      String[] data=loadStrings("C:/Users/Kai/Documents/GitHub/masterThesis/results/resultsOpenCVforProcessing.txt"); 
  
  float fx = float(data[0]);
  float fy = float(data[1]);
  float cx = float(data[2]);
  float cy = float(data[3]);
  
  float near = 1.0;
  float far = 5000.0;
  
  PGraphicsOpenGL pg = (PGraphicsOpenGL) g;
  pg.projection.m00 = 2*fx/resolutionX;
  pg.projection.m01 = 0.0;
  pg.projection.m02 = (resolutionX - 2*cx)/resolutionX;
  pg.projection.m03 = 0.0;
  pg.projection.m10 = 0.0;
  pg.projection.m11 = 2*fy/resolutionY;
  pg.projection.m12 = (-1)*(resolutionY - 2*cy)/resolutionY;
  pg.projection.m13 = 0.0;
  pg.projection.m20 = 0.0;
  pg.projection.m21 = 0.0;
  pg.projection.m22 = (-1)*(far+near)/(far-near);
  pg.projection.m23 = (-2)*(far*near)/(far-near);
  pg.projection.m30 = 0.0;
  pg.projection.m31 = 0.0;
  pg.projection.m32 = -1.0;
  pg.projection.m33 = 0.0;
  
  
  beginCamera();
  resetMatrix();

   rotateX(radians(180));
   applyMatrix( float(data[7]), float(data[8]), float(data[9]), float(data[4]),
                float(data[10]), float(data[11]),  float(data[12]), float(data[5]),
                float(data[13]), float(data[14]), float(data[15]), float(data[6]),
               0.0, 0.0, 0.0,  1.0);
               
  printMatrix();
 
  endCamera();
  
  

  endPGL();
}

public void draw() {



  background(0);

  //image(kinect.getColorImage(), 0, 0, 320, 240);


  // The geometric transformations will be automatically passed to the shader
  //pushMatrix();
  /*translate(width / 2, height / 2, zval);
  scale(scaleVal, -1 * scaleVal, scaleVal);
  rotate(a, 0.0f, 1.0f, 0.0f);*/

  kinect.setLowThresholdPC(minD);
  kinect.setHighThresholdPC(maxD);

  //render to the openGL object
  pgl = beginPGL();
  sh.bind();


  //obtain the point cloud positions
  FloatBuffer pointCloudBuffer = kinect.getPointCloudColorPos();

  //get the color for each point of the cloud Points
  FloatBuffer colorBuffer      = kinect.getColorChannelBuffer();

  //send the the vertex positions (point cloud) and color down the render pipeline
  //positions are render in the vertex shader, and color in the fragment shader
  vertLoc = pgl.getAttribLocation(sh.glProgram, "vertex");
  pgl.enableVertexAttribArray(vertLoc);
  
  
  //enable drawing to the vertex and color buffer
  colorLoc = pgl.getAttribLocation(sh.glProgram, "color");
  pgl.enableVertexAttribArray(colorLoc);

  int vertData = kinect.WIDTHColor * kinect.HEIGHTColor * 3;

  //vertex
  {
    pgl.bindBuffer(PGL.ARRAY_BUFFER, vertexVboId);
    // fill VBO with data
    pgl.bufferData(PGL.ARRAY_BUFFER, Float.BYTES * vertData, pointCloudBuffer, PGL.DYNAMIC_DRAW);
    // associate currently bound VBO with shader attribute
    pgl.vertexAttribPointer(vertLoc, 3, PGL.FLOAT, false, Float.BYTES * 3, 0);
  }

  // color
  {
    pgl.bindBuffer(PGL.ARRAY_BUFFER, colorVboId);
    // fill VBO with data
    pgl.bufferData(PGL.ARRAY_BUFFER, Float.BYTES * vertData, colorBuffer, PGL.DYNAMIC_DRAW);
    // associate currently bound VBO with shader attribute
    pgl.vertexAttribPointer(colorLoc, 3, PGL.FLOAT, false, Float.BYTES * 3, 0);
  }

  // unbind VBOs
  pgl.bindBuffer(PGL.ARRAY_BUFFER, 0);

  //draw the point cloud as a set of points
  pgl.drawArrays(PGL.POINTS, 0, vertData);

  //disable drawing
  pgl.disableVertexAttribArray(vertLoc);
  pgl.disableVertexAttribArray(colorLoc);

  //close the shader
  sh.unbind();
  //close the openGL object
  endPGL();

  //popMatrix();

  stroke(255, 0, 0);
  text(frameRate, 50, height- 50);
}

public void mousePressed() {

  println(frameRate);
  //  saveFrame();
}

/*public void keyPressed() {
  if (key == 'a') {
    zval +=1;
    println(zval);
  }
  if (key == 's') {
    zval -= 1;
    println(zval);
  }

  if (key == 'z') {
    scaleVal += 1;
    println(scaleVal);
  }
  if (key == 'x') {
    scaleVal -= 1;
    println(scaleVal);
  }

  if (key == 'q') {
    a += 0.1;
    println("angle "+a);
  }
  if (key == 'w') {
    a -= 0.1;
    println("angle "+a);
  }

  if (key == 'c') {
    depthVal -= 0.01;
    println(depthVal);
  }

  if (key == 'v') {
    depthVal += 0.01;
    println(depthVal);
  }
}*/