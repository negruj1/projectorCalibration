
import java.nio.*;
import KinectPV2.KJoint;
import KinectPV2.*;
import processing.core.PVector;

import com.jogamp.opengl.GL2;  // Processing 3
//import javax.media.opengl.GL2;  // Processing 2

float a;

PGraphicsOpenGL pg; 
PGL pgl;
GL2 gl;

KinectPV2 kinect;

float tearPosition=1.0;

float zVal = 1000;
float rotX = PI;
float resolutionX = 1280;
float resolutionY = 800;

void setup() {
  fullScreen(P3D);

  kinect = new KinectPV2(this);
  kinect.enableDepthImg(true);
  kinect.enablePointCloud(true);
  kinect.enableHDFaceDetection(true);
  kinect.enableInfraredImg(true);

  kinect.enableFaceDetection(true);
  kinect.enableColorImg(true); //to draw the color image

  //enable 3d  with (x,y,z) position
  kinect.enableSkeleton3DMap(true);
  
      //String[] data=loadStrings("C:/Users/Kai/Documents/GitHub/masterThesis/results/resultsOpenCVforProcessing.txt"); 
      String[] data=loadStrings(System.getProperty("user.home").replace("\\", "/")+"/Pictures/projectorCalibration/results/resultsOpenCVforProcessing.txt");
  
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

  kinect.init();
  
  
  
  
  
}

void draw() {

  background(0);
  
  pushMatrix();

  /*ArrayList<KSkeleton> skeletonArray =  kinect.getSkeleton3d();

  for (int i = 0; i < skeletonArray.size(); i++) {
    KSkeleton skeleton = (KSkeleton) skeletonArray.get(i);
    if (skeleton.isTracked()) {
      KJoint[] joints = skeleton.getJoints();

      drawHandState(joints[KinectPV2.JointType_HandRight]);
      drawHandState(joints[KinectPV2.JointType_HandLeft]);

      color col  = skeleton.getIndexColor();
      stroke(col);
      drawBody(joints);
    }
  }*/
  
  kinect.generateFaceData();
  
  FloatBuffer pointCloudBuffer = kinect.getPointCloudDepthPos();
  ArrayList<FaceData> faceData =  kinect.getFaceData();
  for (int i = 0; i < faceData.size(); i++) {
    //println("test");
    FaceData faceD = faceData.get(i);
    if (faceD.isFaceTracked()) {
      PVector [] fpi = faceD.getFacePointsInfraredMap();
      FaceFeatures [] faceFeatures = faceD.getFaceFeatures();
      PVector leftEye = infraredTo3dPoints(fpi[0],pointCloudBuffer);
      PVector rightEye = infraredTo3dPoints(fpi[1],pointCloudBuffer);
      PVector nose = infraredTo3dPoints(fpi[2],pointCloudBuffer);
      PVector mouthLeft = infraredTo3dPoints(fpi[3],pointCloudBuffer);
      PVector mouthRight = infraredTo3dPoints(fpi[4],pointCloudBuffer);
      
      Boolean happy = (faceFeatures[0].getState() == 1) ? true : false;
      Boolean unhappy = (faceFeatures[0].getState() == 0) ? true : false;
      Boolean mouthOpen = (faceFeatures[6].getState() == 1) ? true : false;
      Boolean mouthClosed = (faceFeatures[0].getState() == 0) ? true : false;
      
      
      
      if(true){
        color col  = color(100, 100, 255);
        stroke(col);
        PVector tearPos = rightEye.copy();
        tearPos.sub(mouthRight);
        tearPos.mult(tearPosition);
        tearPos.add(mouthRight);    
        PVector awayFromNose = tearPos.copy();
        awayFromNose.sub(nose);
        awayFromNose.mult(Math.min(1.0-tearPosition,tearPosition));
        tearPos.add(awayFromNose);
        strokeWeight(5.0f + (tearPos.z/1000)*15);
        point(tearPos.x, tearPos.y, tearPos.z);
        if(tearPosition>0) tearPosition=tearPosition-0.01;
        else{tearPosition=1.0;}
      }
      
      
      /*for (int j = 0; j < fpi.length; j++) {
          int X = (kinect.WIDTHDepth*(int)fpi[j].y+(int)fpi[j].x)*3;
          int Y = (kinect.WIDTHDepth*(int)fpi[j].y+(int)fpi[j].x)*3+1;
          int Z = (kinect.WIDTHDepth*(int)fpi[j].y+(int)fpi[j].x)*3+2;
          //println(pointCloudBuffer.get(X)*-1000+" "+pointCloudBuffer.get(Y)*1000+" "+pointCloudBuffer.get(Z)*-1000);
          color col  = color(255, 204, 0);
          stroke(col);
          strokeWeight(5.0f + pointCloudBuffer.get(Z)*8);
          point(pointCloudBuffer.get(X)*-1000, pointCloudBuffer.get(Y)*1000, pointCloudBuffer.get(Z)*-1000);
      }*/
    }
  }
  
  
  popMatrix();


  fill(255, 0, 0);
  text(frameRate, 50, 50);
}


void drawBody(KJoint[] joints) {
  drawBone(joints, KinectPV2.JointType_Head, KinectPV2.JointType_Neck);
  drawBone(joints, KinectPV2.JointType_Neck, KinectPV2.JointType_SpineShoulder);
  drawBone(joints, KinectPV2.JointType_SpineShoulder, KinectPV2.JointType_SpineMid);

  drawBone(joints, KinectPV2.JointType_SpineMid, KinectPV2.JointType_SpineBase);
  drawBone(joints, KinectPV2.JointType_SpineShoulder, KinectPV2.JointType_ShoulderRight);
  drawBone(joints, KinectPV2.JointType_SpineShoulder, KinectPV2.JointType_ShoulderLeft);
  drawBone(joints, KinectPV2.JointType_SpineBase, KinectPV2.JointType_HipRight);
  drawBone(joints, KinectPV2.JointType_SpineBase, KinectPV2.JointType_HipLeft);

  // Right Arm    
  drawBone(joints, KinectPV2.JointType_ShoulderRight, KinectPV2.JointType_ElbowRight);
  drawBone(joints, KinectPV2.JointType_ElbowRight, KinectPV2.JointType_WristRight);
  drawBone(joints, KinectPV2.JointType_WristRight, KinectPV2.JointType_HandRight);
  drawBone(joints, KinectPV2.JointType_HandRight, KinectPV2.JointType_HandTipRight);
  drawBone(joints, KinectPV2.JointType_WristRight, KinectPV2.JointType_ThumbRight);

  // Left Arm
  drawBone(joints, KinectPV2.JointType_ShoulderLeft, KinectPV2.JointType_ElbowLeft);
  drawBone(joints, KinectPV2.JointType_ElbowLeft, KinectPV2.JointType_WristLeft);
  drawBone(joints, KinectPV2.JointType_WristLeft, KinectPV2.JointType_HandLeft);
  drawBone(joints, KinectPV2.JointType_HandLeft, KinectPV2.JointType_HandTipLeft);
  drawBone(joints, KinectPV2.JointType_WristLeft, KinectPV2.JointType_ThumbLeft);

  // Right Leg
  drawBone(joints, KinectPV2.JointType_HipRight, KinectPV2.JointType_KneeRight);
  drawBone(joints, KinectPV2.JointType_KneeRight, KinectPV2.JointType_AnkleRight);
  drawBone(joints, KinectPV2.JointType_AnkleRight, KinectPV2.JointType_FootRight);

  // Left Leg
  drawBone(joints, KinectPV2.JointType_HipLeft, KinectPV2.JointType_KneeLeft);
  drawBone(joints, KinectPV2.JointType_KneeLeft, KinectPV2.JointType_AnkleLeft);
  drawBone(joints, KinectPV2.JointType_AnkleLeft, KinectPV2.JointType_FootLeft);
  //println("joints");
  drawJoint(joints, KinectPV2.JointType_HandTipLeft);
  drawJoint(joints, KinectPV2.JointType_HandTipRight);
  drawJoint(joints, KinectPV2.JointType_FootLeft);
  drawJoint(joints, KinectPV2.JointType_FootRight);

  drawJoint(joints, KinectPV2.JointType_ThumbLeft);
  drawJoint(joints, KinectPV2.JointType_ThumbRight);

  drawJoint(joints, KinectPV2.JointType_Head);
}

void drawJoint(KJoint[] joints, int jointType) {
  strokeWeight(2.0f + joints[jointType].getZ()*8);
  point(joints[jointType].getX()*-1000, joints[jointType].getY()*1000, joints[jointType].getZ()*-1000);
  //println(joints[jointType].getX()*-1000, joints[jointType].getY()*1000, joints[jointType].getZ()*-1000);
}

void drawBone(KJoint[] joints, int jointType1, int jointType2) {
  strokeWeight(2.0f + joints[jointType1].getZ()*8);
  point(joints[jointType2].getX()*-1000, joints[jointType2].getY()*1000, joints[jointType2].getZ()*-1000);
}

void drawHandState(KJoint joint) {
  handState(joint.getState());
  strokeWeight(5.0f + joint.getZ()*8);
  point(joint.getX()*-1000, joint.getY()*1000, joint.getZ()*-1000);
}

void handState(int handState) {
  switch(handState) {
  case KinectPV2.HandState_Open:
    stroke(0, 255, 0);
    break;
  case KinectPV2.HandState_Closed:
    stroke(255, 0, 0);
    break;
  case KinectPV2.HandState_Lasso:
    stroke(0, 0, 255);
    break;
  case KinectPV2.HandState_NotTracked:
    stroke(100, 100, 100);
    break;
  }
}

PVector infraredTo3dPoints(PVector point, FloatBuffer pointCloudBuffer){
  int X = (kinect.WIDTHDepth*(int)point.y+(int)point.x)*3;
  int Y = (kinect.WIDTHDepth*(int)point.y+(int)point.x)*3+1;
  int Z = (kinect.WIDTHDepth*(int)point.y+(int)point.x)*3+2;
  return new PVector(pointCloudBuffer.get(X)*-1000, pointCloudBuffer.get(Y)*1000, pointCloudBuffer.get(Z)*-1000);
}