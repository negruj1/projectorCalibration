/*
 * projectorCalib.cpp
 *
 *  Created on: Oct 24, 2012
 *      Author: cai
 *
 *
 *  compute the projection matrix  P (at least 6 points, and at least 3 points in different z)
 *  1. result:
 *  Projection matrix :   P
 *  Camera intrinsic parameter matrix: M
 *  rotation matrix  : R
 *  translation vector : T
 *
 *  2. question and steps:
 *     1).decompose 1: Mt, Rt, Tt;
 *         projection matrix:  the result directly got from svd is not the correct one, it should be multiply one scales k;
 *         k = 1/ Mt(2,2), which is decomposed from A, intrinsic parameter matrix temp  Mt
 *         then:
 *         New Projection matrix :   P = k * P;
 *
 *          intrinsic parameter matrix M = k * Mt;   WRONG!!!!!!!!!!!!!!!!!!!!!, should decomposition again to get!!!!
 *
 *     2). decompose again with the new projection matrix;
 *         Mc, Rc, Tc (opencv,change to Eigen M, R)
 *         Note:   new intrinsic parameter matrix M not= k * Mt,   ???????, the sign is different;
 *
 *     3). the translation vector from the decomposition Tc is not the one we want
 *         so
 *         P_temp = [R  T]  = M.inv() * P;
 *
 */

#include "projectionMatrix.h"

Calib::Calib()
{
    this->Pt.resize(3, 4);
    this->P.resize(3, 4);
    this->P_temp.resize(3, 4);
}

Calib::~Calib()
{

}

void Calib::readData()
{
    // input 3D World positions for simulation
    Eigen::Vector3d p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11, p12, p13, p14, p15;

    p1 << 0.1, 0.8, 0;
    p2 << 0.3, 0.42, 0.0;
    p3 << 0.8, 0.93, 0.3;
    objectPoints.push_back(p1);
    objectPoints.push_back(p2);
    objectPoints.push_back(p3);

    p4 << 0.2, 0.7, 0.0;
    p5 << 0.3, 0.29, 0.2;
    p6 << 0.9, 0.24, 0.0;
    objectPoints.push_back(p4);
    objectPoints.push_back(p5);
    objectPoints.push_back(p6);

    //	p7 << 0.9, 0.2, 0.5;
    //	p8 << 0.18, 0.1, 0.2;
    //	p9 << 0.44, 0.6, 0.7;
    //	objectPoints.push_back(p7);
    //	objectPoints.push_back(p8);
    //	objectPoints.push_back(p9);
    //
    //	p10 << 0.5, 0.18, 0.47;
    //	p11 << 0.22, 1.0, 0.47;
    //	p12 << 0.6, 0.83, 0.2;
    //	objectPoints.push_back(p10);
    //	objectPoints.push_back(p11);
    //	objectPoints.push_back(p12);

    //	p13 << 0.1, 0.45, 0.8;
    //	p14 << 0.2, 0.9, 0.07;
    //	p15 << 0.0, 0.38, 0.25;
    //	objectPoints.push_back(p13);
    //	objectPoints.push_back(p14);
    //	objectPoints.push_back(p15);

    // simulation for get the 2D position
    //simualtion input matrixs
    Eigen::MatrixXd P1(3,4);
    Eigen::Matrix3d K, R1;
    Eigen::Vector3d t1;
    Eigen::MatrixXd P1_temp(3, 4);

    K << 534.68792, 0, 327.16897, 0, 533.70712, 237.49886, 0, 0, 1;
    R1 << -0.098015, -0.994936, -0.022267, -0.030656, 0.025382, -0.999208, 0.994713, -0.097255, -0.032988;
    t1 << -0.89188, 0.522579, 1.86515;

    P1_temp << R1(0, 0), R1(0, 1), R1(0, 2), t1(0), R1(1, 0), R1(1, 1), R1(1, 2), t1(1), R1(2, 0), R1(2, 1), R1(2, 2), t1(2);

    P1 = K * P1_temp;

    std::cout << "***********************************************************************************\n";
    std::cout << "origin projection is = \n" << P1 << std::endl;

    for (unsigned i = 0; i < objectPoints.size(); i++)
    {
        Eigen::Vector2d pos;
        Eigen::Vector3d x;
        Eigen::Vector4d X;
        X << objectPoints[i](0), objectPoints[i](1), objectPoints[i](2), 1;

        x = P1 * X;
        pos << x(0) / x(2), x(1) / x(2);

        imagePoints.push_back(pos);
    }

}




void Calib::computeProjectionMatrix()
{
    std::cout << "1111111111***********************************************************************************\n";
//    for(int i = 0;i < imagePoints.size();i++)
//    {
//        std::cout<<" imagePoints[i]= "<< imagePoints[i]<<std::endl;
//        std::cout<<" objectPoints[i]= "<< objectPoints[i]<<std::endl;
//    }

    if (objectPoints.size() != imagePoints.size())
        return;

    int n = objectPoints.size();

    Eigen::Vector2d avg2;
    Eigen::Vector3d avg3;

    for (unsigned int i = 0; i < n; i++)
    {
        avg2 += imagePoints[i];
        avg3 += objectPoints[i];
        std::cout << i << " " << objectPoints[i](0) << " " << objectPoints[i](1) << " " << objectPoints[i](2) << std::endl;
    }
    avg2 = avg2 / n;
    avg3 = avg3 / n;

    std::cout << "avg2 = " << avg2 << std::endl;
    std::cout << "avg3 = " << avg3 << std::endl;

    /* *******************************************************************************
  *  Data normalization
  *  Translates and normalises a set of 2D homogeneous points so that their centroid is at the origin and their mean distance from the origin is sqrt(2).
  */
    float meandist2 = 0;
    float meandist3 = 0;

    imagePoints_t.resize(n);
    objectPoints_t.resize(n);

    for (unsigned int i = 0; i < n; i++)
    {
        imagePoints_t[i] = imagePoints[i] - avg2;
        objectPoints_t[i] = objectPoints[i] - avg3;
//        std::cout << "1 imagePoints_t[i] = " << imagePoints_t[i] << std::endl;
//        std::cout << "1 objectPoints_t[i] = " << objectPoints_t[i] << std::endl;

        meandist2 += sqrt(imagePoints_t[i](0) * imagePoints_t[i](0) + imagePoints_t[i](1) * imagePoints_t[i](1));
        meandist3 += sqrt(objectPoints_t[i](0) * objectPoints_t[i](0) + objectPoints_t[i](1) * objectPoints_t[i](1)
                          + objectPoints_t[i](2) * objectPoints_t[i](2));
    }
    meandist2 /= n;
    meandist3 /= n;

    std::cout << "meandist2 = " << meandist2 << std::endl;
    std::cout << "meandist3 = " << meandist3 << std::endl;

    float scale2 = sqrt(2) / meandist2;
    float scale3 = sqrt(3) / meandist3;


    std::cout << "2222222222222***********************************************************************************\n";
    for (unsigned int i = 0; i < n; i++)
    {
        imagePoints_t[i] = scale2 * imagePoints_t[i];
        objectPoints_t[i] = scale3 * objectPoints_t[i];

//        std::cout << "imagePoints_t[i] = " << imagePoints_t[i] << std::endl;
//        std::cout << "objectPoints_t[i] = " << objectPoints_t[i] << std::endl;

    }

    //    std::cout<<avg3<<std::endl;
    /* *******************************************************************************
  * Similarity transformation T1 to normalize the image points,
  * and a second similarity transformation T2 to normalize the space points.
  * Page 181 in Multiple_View_Geometry_in_Computer_Vision__2nd_Edition
  */
    Eigen::Matrix3d T1;
    T1 << scale2, 0, -scale2 * avg2(0), 0, scale2, -scale2 * avg2(1), 0, 0, 1;

    Eigen::MatrixXd T2(4, 4);
    T2 << scale3, 0, 0, -scale3 * avg3(0), 0, scale3, 0, -scale3 * avg3(1), 0, 0, scale3, -scale3 * avg3(2), 0, 0, 0, 1;


    // use Eigen
    Eigen::MatrixXd A(2 * n, 12);
    A.setZero(2 * n, 12);

    for (int i = 0; i < n; i++)
    {

        A(2 * i, 0) = objectPoints_t[i](0);
        A(2 * i, 1) = objectPoints_t[i](1);
        A(2 * i, 2) = objectPoints_t[i](2);
        A(2 * i, 3) = 1;
        A(2 * i, 4) = 0;
        A(2 * i, 5) = 0;
        A(2 * i, 6) = 0;
        A(2 * i, 7) = 0;
        A(2 * i, 8) = -imagePoints_t[i](0) * objectPoints_t[i](0);
        A(2 * i, 9) = -imagePoints_t[i](0) * objectPoints_t[i](1);
        A(2 * i, 10) = -imagePoints_t[i](0) * objectPoints_t[i](2);
        A(2 * i, 11) = -imagePoints_t[i](0) * 1;

        A(2 * i + 1, 0) = 0;
        A(2 * i + 1, 1) = 0;
        A(2 * i + 1, 2) = 0;
        A(2 * i + 1, 3) = 0;
        A(2 * i + 1, 4) = 1 * objectPoints_t[i](0);
        A(2 * i + 1, 5) = 1 * objectPoints_t[i](1);
        A(2 * i + 1, 6) = 1 * objectPoints_t[i](2);
        A(2 * i + 1, 7) = 1;
        A(2 * i + 1, 8) = -imagePoints_t[i](1) * objectPoints_t[i](0);
        A(2 * i + 1, 9) = -imagePoints_t[i](1) * objectPoints_t[i](1);
        A(2 * i + 1, 10) = -imagePoints_t[i](1) * objectPoints_t[i](2);
        A(2 * i + 1, 11) = -imagePoints_t[i](1) * 1;

    }

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
    const Eigen::VectorXd & v1 = svd.matrixV().col(11);

    this->Pt << v1(0), v1(1), v1(2), v1(3), v1(4), v1(5), v1(6), v1(7), v1(8), v1(9), v1(10), v1(11);
//    std::cout<<"A= \n"<< A<<std::endl;
//    std::cout<<Pt<<std::endl;
    P = T1.inverse() * Pt * T2;


    //Decompose the projection matrix
    cv::Mat Pr(3, 4, cv::DataType<float>::type);
    cv::Mat Mt(3, 3, cv::DataType<float>::type);
    cv::Mat Rt(3, 3, cv::DataType<float>::type);
    cv::Mat Tt(4, 1, cv::DataType<float>::type);

    for (unsigned i = 0; i < 3; i++)
    {
        for (unsigned int j = 0; j < 4; j++)
        {
            Pr.at<float> (i, j) = P(i, j);
        }
    }
    cv::decomposeProjectionMatrix(Pr, Mt, Rt, Tt);

    //scale: Mt(2,2) should = 1; so update the projection matrix and decomposition again
    float k = (1 / (Mt.at<float> (2, 2)));

    /* ****************************************************************************************
      * Upate the projection matrix
      * Decomposition again to get new intrinsic matrix and rotation matrix
      */
    this->P = k * P;

    cv::Mat Pro(3, 4, cv::DataType<float>::type);
    cv::Mat Mc(3, 3, cv::DataType<float>::type); // intrinsic parameter matrix
    cv::Mat Rc(3, 3, cv::DataType<float>::type); // rotation matrix
    cv::Mat Tc(4, 1, cv::DataType<float>::type); // translation vector

    for (unsigned i = 0; i < 3; i++)
    {
        for (unsigned int j = 0; j < 4; j++)
        {
            Pro.at<float> (i, j) = P(i, j);
        }
    }
    cv::decomposeProjectionMatrix(Pro, Mc, Rc, Tc);

    // Change from OpenCV varibles to Eigen
    for (unsigned i = 0; i < 3; i++)
    {
        for (unsigned int j = 0; j < 3; j++)
        {
            M(i, j) = Mc.at<float> (i, j) ;
        }
    }


    /* ****************************************************************************************
      * Compute te rotation matrix R and translation vector T
      */
    P_temp = M.inverse() * P;
    this->R = this->P_temp.block(0,0,3,3);
    this->T = this->P_temp.block(0,3,3,1);


    std::cout << "+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++\n";
//    std::cout << "T1 =\n " << T2 << std::endl;
//    std::cout << "T2 =\n " << T1 << std::endl;
//    std::cout << "A =\n " << A << std::endl;
//    std::cout << "svd.matrixV() =\n " << svd.matrixV() << std::endl;
//    std::cout << "Pt =\n " << Pt << std::endl;
    std::cout << "P =\n " << P << std::endl;
    std::cout << "M =\n " << M << std::endl;
    std::cout << "R =\n " << R << std::endl;
    std::cout << "Rc =\n " << Rc << std::endl;
    std::cout << "T =\n " << T << std::endl;
    std::cout << "Mt(2,2) = " << Mt.at<float> (2, 2) << std::endl;

}

void Calib::computeCameraPosePnP()
{
    std::cout << "There are " << imagePoints.size() << " imagePoints and " << objectPoints.size() << " objectPoints." << std::endl;
    cv::Mat cameraMatrix(3,3,cv::DataType<double>::type);
    cv::setIdentity(cameraMatrix);
    for (unsigned i = 0; i < 3; i++)
    {
        for (unsigned int j = 0; j < 3; j++)
        {
            cameraMatrix.at<double> (i, j) = M(i, j);
        }
    }
    std::cout << "Initial cameraMatrix: " << cameraMatrix << std::endl;

    cv::Mat distCoeffs(4,1,cv::DataType<double>::type);
    distCoeffs.at<double>(0) = 0;
    distCoeffs.at<double>(1) = 0;
    distCoeffs.at<double>(2) = 0;
    distCoeffs.at<double>(3) = 0;

    cv::Mat rvec(3,1,cv::DataType<double>::type);
    cv::Mat tvec(3,1,cv::DataType<double>::type);

    std::vector<cv::Point3d> objectPoints_cv;
    std::vector<cv::Point2d> imagePoints_cv;

    for(int point_id = 0;point_id < objectPoints.size();point_id++)
    {
        cv::Point3d object_point;
        cv::Point2d image_point;
        object_point.x = objectPoints[point_id](0);
        object_point.y = objectPoints[point_id](1);
        object_point.z = objectPoints[point_id](2);

        image_point.x = imagePoints[point_id](0);
        image_point.y = imagePoints[point_id](1);

        objectPoints_cv.push_back(object_point);
        imagePoints_cv.push_back(image_point);
    }
    cv::solvePnP(objectPoints_cv, imagePoints_cv, cameraMatrix, distCoeffs, rvec, tvec);

    std::cout << "rvec: " << rvec << std::endl;
    std::cout << "tvec: " << tvec << std::endl;

    std::vector<cv::Point2d> projectedPoints;
    cv::projectPoints(objectPoints_cv, rvec, tvec, cameraMatrix, distCoeffs, projectedPoints);

    for(unsigned int i = 0; i < projectedPoints.size(); ++i)
    {
        std::cout << "Image point: " << imagePoints[i] << " Projected to " << projectedPoints[i] << std::endl;
    }

    //inverting the R|t to get camera position in world co-ordinates
    cv::Mat Rot(3,3,cv::DataType<double>::type);
    cv::Rodrigues(rvec, Rot);
    Rot = Rot.t();
    tvec = -Rot*tvec;
    this->R << Rot.at<double>(0,0), Rot.at<double>(0,1), Rot.at<double>(0,2), Rot.at<double>(1,0), Rot.at<double>(1,1), Rot.at<double>(1,2), Rot.at<double>(2,0), Rot.at<double>(2,1), Rot.at<double>(2,2);
    this->T << tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2);
}

double Calib::compute_reprojection_error()
{
    double error = 0;
    for(int point_id = 0;point_id < objectPoints.size();point_id++)
    {
        Eigen::Vector3d image_point_ref;
        image_point_ref << imagePoints[point_id], 1;
        Eigen::Vector4d object_point;
        object_point << objectPoints[point_id], 1;
        Eigen::Vector3d image_point_est = P*object_point;
        image_point_est /= image_point_est(2);
        error += (image_point_ref-image_point_est).transpose()*(image_point_ref-image_point_est);
    }
    error /= objectPoints.size();
    error = std::sqrt(error);
    return error;
}

//int main(int argc, char** argv)
//{

//	Calib project;
//	project.readData();
//	project.computeProjectionMatrix();

//	return 0;
//}


