import gab.opencv.*;
import KinectPV2.*;
import com.jogamp.opengl.GL2;

KinectPV2 kinect;
OpenCV opencv;

PGraphicsOpenGL pg; 
GL2 gl;

float polygonFactor = 1;

int threshold = 10;

//Distance in cm
int maxD = 4500; //4.5m
int minD = 50; //50cm
float resolutionX = 1280;
float resolutionY = 800;

boolean    contourBodyIndex = true;

void setup() {
  fullScreen(P3D);
  opencv = new OpenCV(this, 512, 424);
  kinect = new KinectPV2(this);

  kinect.enableDepthImg(true);
  kinect.enableBodyTrackImg(true);
  kinect.enablePointCloud(true);

  kinect.init();
  
  String[] data=loadStrings(System.getProperty("user.home").replace("\\", "/")+"/Pictures/projectorCalibration/results/resultsOpenCVforProcessing.txt");
  //String[] data=loadStrings("C:/Users/Kai/Documents/GitHub/masterThesis/results/resultsOpenCVforProcessing.txt"); 
  
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
  
}

void draw() {
  background(0);

  noFill();
  strokeWeight(3);

  //image(kinect.getDepthImage(), 0, 0);
  
    opencv.loadImage(kinect.getBodyTrackImage());
    opencv.gray();
    opencv.threshold(threshold);
    PImage dst = opencv.getOutput();


  ArrayList<Contour> contours = opencv.findContours(false, false);

  if (contours.size() > 0) {
    for (Contour contour : contours) {

      contour.setPolygonApproximationFactor(polygonFactor);
      if (contour.numPoints() > 50) {

        stroke(0, 200, 200);
        beginShape();

        for (PVector point : contour.getPolygonApproximation ().getPoints()) {
          vertex(point.x , point.y, point.z);
        }
        endShape();
      }
    }
  }

  noStroke();
  fill(0);
  //rect(0, 0, 130, 100);
  //fill(255, 0, 0);

  kinect.setLowThresholdPC(minD);
  kinect.setHighThresholdPC(maxD);
}


/*void keyPressed() {
  //change contour finder from contour body to depth-PC
  if( key == 'b'){
   contourBodyIndex = !contourBodyIndex;
   if(contourBodyIndex)
     threshold = 200;
    else
     threshold = 40;
  }

  if (key == 'a') {
    threshold+=1;
  }
  if (key == 's') {
    threshold-=1;
  }

  if (key == '1') {
    minD += 10;
  }

  if (key == '2') {
    minD -= 10;
  }

  if (key == '3') {
    maxD += 10;
  }

  if (key == '4') {
    maxD -= 10;
  }

  if (key == '5')
    polygonFactor += 0.1;

  if (key == '6')
    polygonFactor -= 0.1;
}*/