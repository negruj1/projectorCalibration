////////////////////////////////////////////////////////////////////////
//
// hello-world.cpp
//
// This is a simple, introductory OpenCV program. The program reads an
// image from a file, inverts it, and displays the result. 
//
////////////////////////////////////////////////////////////////////////
#define _CRT_SECURE_NO_DEPRECATE

#include <iostream>
#include <stdio.h>



#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <cv.h>
#include <highgui.h>
#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <Eigen/Geometry>
#include <fstream>
#include <string>
#include <shlobj.h>
#include <direct.h>
#include <shlobj.h>
#include <sstream>
#include <tchar.h>

using namespace Eigen;
using namespace std;
using namespace cv;

typedef std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> > Point2DVector;
typedef std::vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d> > Point3DVector;

int main ( int argc, char **argv )
{

	TCHAR mypicturespath[MAX_PATH];
    HRESULT result = SHGetFolderPath(NULL, CSIDL_MYPICTURES, NULL, SHGFP_TYPE_CURRENT, mypicturespath);        
	ofstream file;  
    std::basic_ostringstream<TCHAR> file_path;
    file_path << mypicturespath << _TEXT("\\projectorCalibration\\results\\numberOfPoints.txt");
	wcout << file_path.str().c_str() << endl;
	std::fstream myfile4(file_path.str().c_str(), std::ios_base::in);
	//std::fstream myfile4("C:\\Users\\Kai\\Documents\\GitHub\\masterThesis\\results\\numberOfPoints.txt", std::ios_base::in);
	
	float b;
	int n = 30;
	while(myfile4 >> b){
		n= (int)b;
	}

	int ransacSampleSize=2;//80;
	int ransacIterations=50;
  
	Point2DVector  imagePoints;
    Point3DVector  objectPoints;

	Point2DVector  imagePointsTotal;
    Point3DVector  objectPointsTotal;

    Eigen::Vector2d avg2(0,0);
    Eigen::Vector3d avg3(0,0,0);
	Point2DVector imagePoints_t;
    Point3DVector objectPoints_t;

	Eigen::MatrixXd P_temp;

	Eigen::MatrixXd P;
    Eigen::Matrix3d R;
    Eigen::Vector3d T;
    Eigen::Matrix3d M;

	TCHAR mypicturespath2[MAX_PATH];
    HRESULT result2 = SHGetFolderPath(NULL, CSIDL_MYPICTURES, NULL, SHGFP_TYPE_CURRENT, mypicturespath2);        
	ofstream file2;  
    std::basic_ostringstream<TCHAR> file_path2;
    file_path2 << mypicturespath2 << _TEXT("\\projectorCalibration\\results\\first_results_1.txt");
	wcout << file_path2.str().c_str() << endl;
	std::fstream myfile(file_path2.str().c_str(), std::ios_base::in);
	
	//std::fstream myfile("C:\\Users\\Kai\\Documents\\GitHub\\masterThesis\\results\\first_results_1.txt", std::ios_base::in);

    float a;
	int counter = 0;
	float u = 0;
	float v = 0;
	float x = 0;
	float y = 0;
	float z = 0;
    while (myfile >> a)
    {
		if(counter==0){
			u = a;
			counter++;
		}
		else if(counter==1){
			v = a;
			Eigen::Vector2d point2d(u,v);
			imagePointsTotal.push_back(point2d);
			counter++;
		}
		else if(counter==2){
			x=a;
			counter++;
		}
		else if(counter==3){
			y=a;
			counter++;
		}
		else if(counter==4){
			z=a;
			Eigen::Vector3d point3d(x,y,z);
			objectPointsTotal.push_back(point3d);
			counter=0;
		}
		else{
			cout<<"something went wrong"<<endl;
		}
        //printf("%f ", a);
    }

	vector<int> choosenPointsWithLeastError;
	double minError = 1000000.0;
	int maxPointsIn=0;
	Eigen::Matrix3d bestR;
    Eigen::Vector3d bestT;
    Eigen::Matrix3d bestM;

	srand (time(NULL));

	for(int ransacIter=0;ransacIter<ransacIterations;ransacIter++){
	
		objectPoints.clear();
		imagePoints.clear();
	
	

	vector<int> choosenPoints;
	for(int i=0;i<objectPointsTotal.size();i++){
		choosenPoints.push_back(i);
	}
	for(int i =0;i<ransacSampleSize;i++){
		int randomIndividual = rand() % (choosenPoints.size());
		objectPoints.push_back(objectPointsTotal.at(choosenPoints.at(randomIndividual)));
		imagePoints.push_back(imagePointsTotal.at(choosenPoints.at(randomIndividual)));
		choosenPoints.erase(choosenPoints.begin() + randomIndividual);
	}

	n=objectPoints.size();

	//cout<<"Length:"<<objectPoints.size()<<endl;

    for (unsigned int i = 0; i < n; i++)
    {
		//std::cout << i << " " << avg2 << std::endl;
        avg2 += imagePoints[i];
        avg3 += objectPoints[i];
		//std::cout << i << " " << imagePoints[i](0) << " " << imagePoints[i](1) << std::endl;
        //std::cout << i << " " << objectPoints[i](0) << " " << objectPoints[i](1) << " " << objectPoints[i](2) << std::endl;
		
    }
    avg2 = avg2 / (n*1.0);
    avg3 = avg3 / (n*1.0);

    //std::cout << "avg2 = " << avg2 << std::endl;
    //std::cout << "avg3 = " << avg3 << std::endl;

	

	float meandist2 = 0;
    float meandist3 = 0;

    imagePoints_t.resize(n);
    objectPoints_t.resize(n);

	//cout<<"Length2:"<<objectPoints.size()<<endl;

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

	//cout<<"Length3:"<<objectPoints.size()<<endl;

    meandist2 /= n;
    meandist3 /= n;

    //std::cout << "meandist2 = " << meandist2 << std::endl;
    //std::cout << "meandist3 = " << meandist3 << std::endl;

    float scale2 = sqrt(2) / meandist2;
    float scale3 = sqrt(3) / meandist3;

	//std::cout << "scale2 = " << scale2 << std::endl;
    //std::cout << "scale3 = " << scale3 << std::endl;


    //std::cout << "2222222222222***********************************************************************************\n";
    for (unsigned int i = 0; i < n; i++)
    {
        imagePoints_t[i] = scale2 * imagePoints_t[i];
        objectPoints_t[i] = scale3 * objectPoints_t[i];

        //std::cout << "imagePoints_t[i] = " << imagePoints_t[i] << std::endl;
        //std::cout << "objectPoints_t[i] = " << objectPoints_t[i] << std::endl;

    }

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

	//cout<<v1<<endl;

    Eigen::MatrixXd Pt (3,4);
		
	Pt << v1(0), v1(1), v1(2), v1(3), v1(4), v1(5), v1(6), v1(7), v1(8), v1(9), v1(10), v1(11);
    
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
    P = k * P;

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
    R = P_temp.block(0,0,3,3);
    T = P_temp.block(0,3,3,1);


	double squaredReprojectionError=0.0;

	/*for (int i =0;i<n;i++){
		double u = imagePoints[i].x();
		double v = imagePoints[i].y();

		double x = objectPoints[i].x();
		double y = objectPoints[i].y();
		double z = objectPoints[i].z();


		Vector4d myVec(x,y,z,1.0);
		Vector3d result = P*myVec;
		double myU = result.x()/result.z();
		double myV = result.y()/result.z();

		squaredReprojectionError+= (myU-u)*(myU-u)+(myV-v)*(myV-v);

	}*/

	int numberOfPointsIn=0;

	for (int i =0;i<objectPointsTotal.size();i++){
		double u = imagePointsTotal[i].x();
		double v = imagePointsTotal[i].y();

		double x = objectPointsTotal[i].x();
		double y = objectPointsTotal[i].y();
		double z = objectPointsTotal[i].z();


		Vector4d myVec(x,y,z,1.0);
		Vector3d result = P*myVec;
		double myU = result.x()/result.z();
		double myV = result.y()/result.z();


		if(sqrt((myU-u)*(myU-u)+(myV-v)*(myV-v))<3.0){
			numberOfPointsIn++;
		}

		//cout<<"Reprojection: "<< myU<<" "<<myV<<" "<<myU-u<<" "<<myV-v<<" "<<endl;
		//cout<<"ProjectionMatrixes: "<<endl<< P<<" "<<endl<<endl<<rebuiltProjectionMatrix3x4<<endl<<endl<<ext<<" "<<endl;
	}

	cout<<"squaredReprojectionError: "<< squaredReprojectionError<<" n:"<<objectPointsTotal.size()<<endl;

	cout<<"numberOfPointsIn: "<< numberOfPointsIn<<" n:"<<objectPointsTotal.size()<<endl;

	/*if(squaredReprojectionError < minError){
		minError = squaredReprojectionError;
		choosenPointsWithLeastError = choosenPoints;
		bestM=M;
		bestT=T;
		bestR=R;
	}*/

	if(numberOfPointsIn > maxPointsIn){
		maxPointsIn = numberOfPointsIn;
		choosenPointsWithLeastError = choosenPoints;
		bestM=M;
		bestT=T;
		bestR=R;
	}

	


	/*std::cout << "+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++\n";
    std::cout << "T1 =\n " << T2 << std::endl;
    std::cout << "T2 =\n " << T1 << std::endl;
    std::cout << "Pt =\n " << Pt << std::endl;
    std::cout << "P =\n " << P << std::endl;
    std::cout << "M =\n " << M << std::endl;
    std::cout << "R =\n " << R << std::endl;
    std::cout << "Rc =\n " << Rc << std::endl;
    std::cout << "T =\n " << T << std::endl;
    std::cout << "Mt(2,2) = " << Mt.at<float> (2, 2) << std::endl;*/

	ofstream myfile2;
	TCHAR mypicturespath3[MAX_PATH];
    HRESULT result3 = SHGetFolderPath(NULL, CSIDL_MYPICTURES, NULL, SHGFP_TYPE_CURRENT, mypicturespath3);        
	ofstream file3;  
    std::basic_ostringstream<TCHAR> file_path3;
    file_path3 << mypicturespath3 << _TEXT("\\projectorCalibration\\results\\resultsOpenCV.txt");
	//wcout << file_path3.str().c_str() << endl;
	myfile2.open (file_path3.str().c_str());
	myfile2 <<"M =\n " << M <<"\n";
	myfile2 <<"\n";
	myfile2 <<"T =\n " << T <<"\n";
	myfile2 <<"\n";
	myfile2 <<"R =\n " << R <<"\n";
	
	
	myfile2.close();

	ofstream myfile3;
	TCHAR mypicturespath4[MAX_PATH];
    HRESULT result4 = SHGetFolderPath(NULL, CSIDL_MYPICTURES, NULL, SHGFP_TYPE_CURRENT, mypicturespath4);        
	ofstream file4;  
    std::basic_ostringstream<TCHAR> file_path4;
    file_path4 << mypicturespath4 << _TEXT("\\projectorCalibration\\results\\resultsOpenCVforProcessing.txt");
	//wcout << file_path4.str().c_str() << endl;
	myfile3.open (file_path4.str().c_str());
	myfile3 << M(0,0) <<"\n";
	myfile3 << M(1,1) <<"\n";
	myfile3 << M(0,2) <<"\n";
	myfile3 << M(1,2) <<"\n";
	myfile3 << T(0) <<"\n";
	myfile3 << T(1) <<"\n";
	myfile3 << T(2) <<"\n";
	myfile3 << R(0,0) <<"\n";
	myfile3 << R(0,1) <<"\n";
	myfile3 << R(0,2) <<"\n";
	myfile3 << R(1,0) <<"\n";
	myfile3 << R(1,1) <<"\n";
	myfile3 << R(1,2) <<"\n";
	myfile3 << R(2,0) <<"\n";
	myfile3 << R(2,1) <<"\n";
	myfile3 << R(2,2) <<"\n";
	myfile3 << 0.0 <<"\n";
	myfile3 << 0.0 <<"\n";
	myfile3 << 0.0 <<"\n";
	myfile3 << 0.0 <<"\n";
	myfile3 << 1.0 <<"\n";//as f
	
	
	myfile3.close();


	}

	/*ofstream myfile5;
	myfile5.open ("C:\\Users\\Kai\\Documents\\GitHub\\masterThesis\\results\\resultsOpenCVforProcessingUsingRansac.txt");
	myfile5 << bestM(0,0) <<"\n";
	myfile5 << bestM(1,1) <<"\n";
	myfile5 << bestM(0,2) <<"\n";
	myfile5 << bestM(1,2) <<"\n";
	myfile5 << bestT(0) <<"\n";
	myfile5 << bestT(1) <<"\n";
	myfile5 << bestT(2) <<"\n";
	myfile5 << bestR(0,0) <<"\n";
	myfile5 << bestR(0,1) <<"\n";
	myfile5 << bestR(0,2) <<"\n";
	myfile5 << bestR(1,0) <<"\n";
	myfile5 << bestR(1,1) <<"\n";
	myfile5 << bestR(1,2) <<"\n";
	myfile5 << bestR(2,0) <<"\n";
	myfile5 << bestR(2,1) <<"\n";
	myfile5 << bestR(2,2) <<"\n";
	myfile5 << 0.0 <<"\n";
	myfile5 << 0.0 <<"\n";
	myfile5 << 0.0 <<"\n";
	myfile5 << 0.0 <<"\n";
	myfile5 << 1.0 <<"\n";//as f
	
	
	myfile5.close();

	ofstream myfile6;
	myfile6.open ("C:\\Users\\Kai\\Documents\\GitHub\\masterThesis\\results\\ransacSampleSize.txt");
	myfile6 << ransacSampleSize <<"\n";
	myfile6.close();

	ofstream myfile7;
	myfile7.open ("C:\\Users\\Kai\\Documents\\GitHub\\masterThesis\\results\\ransacPoints_1.txt");
	for(int i=0;i<ransacSampleSize;i++){
		int x = choosenPointsWithLeastError[i];
		Vector2d uv =  imagePointsTotal[x];
		Vector3d xyz =  objectPointsTotal[x];
		myfile7 << uv.x() <<" "<< uv.y() <<" "<< xyz.x() <<" "<< xyz.y() <<" "<< xyz.z() <<" "<<"\n";
	}
	myfile7.close();


	*/
	cout<<"maxPointsIn: "<< maxPointsIn<<" "<<endl;

    std::cout << "M =\n " << bestM << std::endl;
    std::cout << "R =\n " << bestR << std::endl;
    std::cout << "T =\n " << bestT << std::endl;
	/*

	ofstream myfile8;
	myfile8.open ("C:\\Users\\Kai\\Documents\\GitHub\\masterThesis\\results\\strengthOfRansac.txt");
	for(int i=1;i<10;i++){
		double p = (ransacSampleSize*1.0)/(objectPointsTotal.size()*1.0);//probability for 1 bad datapoint to be in one ransacsample
		double q = pow (1.0-p, (i * 1.0));//probability that none of i bad datapoints are in 1 ransacsample
		std::cout << "Probability of " << i << " outliers being filtered out: "<<  1.0 - pow ((1.0-q) , (ransacIterations * 1.0)) << std::endl;
		myfile8 << "Probability of " << i << " outliers being filtered out: "<<  1.0 - pow ((1.0-q) , (ransacIterations * 1.0)) <<"\n";
	}
	myfile8 << "maxPointsIn " << maxPointsIn <<"\n";
	myfile8.close();*/

	getchar();

  return 0;
}