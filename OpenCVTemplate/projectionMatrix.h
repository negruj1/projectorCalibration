/*
 * projectorCalib.h
 *
 *  Created on: Oct 24, 2012
 *      Author: cai
 */

#ifndef PROJECTORCALIB_H_
#define PROJECTORCALIB_H_


#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <iostream>
#include <stdio.h>
#include <vector>
#include <Eigen/Dense>
#include <eigen3/Eigen/StdVector>
#include <Eigen/Geometry>

//using namespace std;
using namespace cv;

typedef std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> > Point2DVector;
typedef std::vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d> > Point3DVector;

class Calib
{
public:
    Calib();
    ~Calib();

    void readData();
    void computeProjectionMatrix();
    void computeCameraPosePnP();
    double compute_reprojection_error();

    Point2DVector imagePoints;
    Point3DVector objectPoints;

    Eigen::MatrixXd P;
    Eigen::Matrix3d R;
    Eigen::Vector3d T;
    Eigen::Matrix3d M;

private:

    //Original Points

    // Points after normalization
    Point2DVector imagePoints_t;
    Point3DVector objectPoints_t;


    Eigen::MatrixXd Pt,P_temp;

};



#endif /* PROJECTORCALIB_H_ */

