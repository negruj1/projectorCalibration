/*
Thomas Sanchez Lengeling.
 http://codigogenerativo.com/

 KinectPV2, Kinect for Windows v2 library for processing

 HD Face tracking.
 Vertex Face positions are mapped to the Color Frame or to the Infrared Frame
 */
 import java.nio.*;
import KinectPV2.*;

KinectPV2 kinect;

void setup() {
  size(1920, 1080);
  

  kinect = new KinectPV2(this);
  kinect.enableDepthImg(true);

  kinect.enablePointCloud(true);

  //enable HD Face detection
  kinect.enableHDFaceDetection(true);
  kinect.enableInfraredImg(true);

  kinect.enableFaceDetection(true);
  kinect.enableColorImg(true); //to draw the color image
  kinect.init();
}

void draw() {
  background(0);
  
  
  
  //mapColorPixelsTo3dPoints();
  // Draw the color Image
  image(kinect.getColorImage(), 0, 0);
  //Obtain the Vertex Face Points
  // 1347 Vertex Points for each user.
  ArrayList<HDFaceData> hdFaceData = kinect.getHDFaceVertex();
  
  kinect.generateFaceData();
  
  FloatBuffer pointCloudBuffer = kinect.getPointCloudDepthPos();
  ArrayList<FaceData> faceData =  kinect.getFaceData();
  for (int i = 0; i < faceData.size(); i++) {
    println("test");
    FaceData faceD = faceData.get(i);
    if (faceD.isFaceTracked()) {
      PVector [] facePointsInfrared = faceD.getFacePointsInfraredMap();
      println(facePointsInfrared.length);
      for (int j = 0; j < facePointsInfrared.length; j++) {
        println(facePointsInfrared[j].y+","+facePointsInfrared[j].x);
          int X = (kinect.WIDTHDepth*(int)facePointsInfrared[j].y+(int)facePointsInfrared[j].x)*3;
          int Y = (kinect.WIDTHDepth*(int)facePointsInfrared[j].y+(int)facePointsInfrared[j].x)*3+1;
          int Z = (kinect.WIDTHDepth*(int)facePointsInfrared[j].y+(int)facePointsInfrared[j].x)*3+2;
          println(pointCloudBuffer.get(X)+" "+pointCloudBuffer.get(Y)+" "+pointCloudBuffer.get(Z));
      }
    }
  }
  

  for (int j = 0; j < hdFaceData.size(); j++) {
    //obtain a the HDFace object with all the vertex data
    HDFaceData HDfaceData = (HDFaceData)hdFaceData.get(j);

    if (HDfaceData.isTracked()) {

      //draw the vertex points
      stroke(0, 255, 0);
      beginShape(POINTS);
      for (int i = 0; i < KinectPV2.HDFaceVertexCount; i++) {
        
        float x = HDfaceData.getX(i);
        float y = HDfaceData.getY(i);
        vertex(x, y);
      }
      endShape();
    }
  }
  

  
  
}

void mapColorPixelsTo3dPoints(){
  FloatBuffer pointCloudBuffer = kinect.getPointCloudDepthPos();

   //obtain XYZ the values of the point cloud
   for(int i = 0; i < kinect.WIDTHDepth * kinect.HEIGHTDepth; i++){
      float x = pointCloudBuffer.get(i*3 + 0);
      float y = pointCloudBuffer.get(i*3 + 1);
      float z = pointCloudBuffer.get(i*3 + 2);
      //print(x+" "+y+" "+z);
   }
}