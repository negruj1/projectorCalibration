
import KinectPV2.KJoint;
import KinectPV2.*;

import com.jogamp.opengl.GL2;  // Processing 3
//import javax.media.opengl.GL2;  // Processing 2

float a;

PGraphicsOpenGL pg; 
PGL pgl;
GL2 gl;

KinectPV2 kinect;


float zVal = 1000;
float rotX = PI;
float resolutionX = 1280;
float resolutionY = 800;

void setup() {
  fullScreen(P3D);

  kinect = new KinectPV2(this);

  kinect.enableColorImg(true);

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
  
  /*pushMatrix();
  translate(0, 0, -2000);
  noFill();
  stroke(255);
  box(280);
  popMatrix();*/

  //image(kinect.getColorImage(), 0, 0, 320, 240);

  //translate the scene to the center 
  pushMatrix();
  //translate(width/2, height/2, 0);
  //scale(zVal);
  //rotateX(rotX);

  ArrayList<KSkeleton> skeletonArray =  kinect.getSkeleton3d();

  //individual JOINTS
  for (int i = 0; i < skeletonArray.size(); i++) {
    KSkeleton skeleton = (KSkeleton) skeletonArray.get(i);
    if (skeleton.isTracked()) {
      KJoint[] joints = skeleton.getJoints();

      //draw different color for each hand state
      drawHandState(joints[KinectPV2.JointType_HandRight]);
      drawHandState(joints[KinectPV2.JointType_HandLeft]);

      //Draw body
      color col  = skeleton.getIndexColor();
      stroke(col);
      drawBody(joints);
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
  println(joints[jointType].getX()*-1000, joints[jointType].getY()*1000, joints[jointType].getZ()*-1000);
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