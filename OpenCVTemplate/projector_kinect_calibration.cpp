#include "projector_kinect_calibration.h"
#include "ui_projector_kinect_calibration.h"

ProjectorKinectCalibration::ProjectorKinectCalibration(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::ProjectorKinectCalibration)
{
    ui->setupUi(this);
    QPixmap pixmap(QSize(64, 64));
    pixmap.fill(QColor::fromRgb(255,255,255));
    cursor = new QCursor(pixmap);

    mWidget = new Widget(this->ui->centralwidget);
    ui->centralwidget->layout()->addWidget(mWidget);

    ui->centralwidget->setStyleSheet("background-color:rgb(255,255,255);");

    calibration_done = false;
    kinect_subscriber = nh.subscribe("/camera/depth_registered/points", 1, &ProjectorKinectCalibration::kinect_callback, this);
    image_transport::ImageTransport it_(nh);
    image_sub_ = it_.subscribe("/camera/rgb/image_raw", 1, &ProjectorKinectCalibration::image_callback, this);
    mWidget->animate();
    this->centralWidget()->update();
    this->timerID = startTimer(10);
    frameCount = 0;
    elapsFrame = 0;

    std::srand ( unsigned ( std::time(0) ) );
}

void ProjectorKinectCalibration::loadExtrinsicsKinect(std::string filename)
{
    QString name(filename.c_str());
    QFileInfo config(name);

    if(!config.exists())
    {
        std::cout<< "error reading calibration file: "<< filename << std::endl;
        throw;
    }

    QSettings iniFile(name, QSettings::IniFormat);

    //parse plan file path
    iniFile.beginGroup("TRANSFORMATION_MATRIX");
    T_kinect_robot(0,0) = iniFile.value("P_00", "1.0").toFloat();
    T_kinect_robot(0,1) = iniFile.value("P_01", "0.0").toFloat();
    T_kinect_robot(0,2) = iniFile.value("P_02", "0.0").toFloat();
    T_kinect_robot(0,3) = iniFile.value("P_03", "0.0").toFloat();
    T_kinect_robot(1,0) = iniFile.value("P_10", "0.0").toFloat();
    T_kinect_robot(1,1) = iniFile.value("P_11", "1.0").toFloat();
    T_kinect_robot(1,2) = iniFile.value("P_12", "0.0").toFloat();
    T_kinect_robot(1,3) = iniFile.value("P_13", "0.0").toFloat();
    T_kinect_robot(2,0) = iniFile.value("P_20", "0.0").toFloat();
    T_kinect_robot(2,1) = iniFile.value("P_21", "0.0").toFloat();
    T_kinect_robot(2,2) = iniFile.value("P_22", "1.0").toFloat();
    T_kinect_robot(2,3) = iniFile.value("P_23", "0.0").toFloat();
    T_kinect_robot(3,0) = iniFile.value("P_30", "0.0").toFloat();
    T_kinect_robot(3,1) = iniFile.value("P_31", "0.0").toFloat();
    T_kinect_robot(3,2) = iniFile.value("P_32", "0.0").toFloat();
    T_kinect_robot(3,3) = iniFile.value("P_33", "1.0").toFloat();
    iniFile.endGroup();

    //    Eigen::Matrix4f correct_axes, T_table_robot;
    //    correct_axes << 0,1,0,0,    1,0,0,0,    0,0,-1,0,    0,0,0,1;
    //    T_kinect_robot = T_kinect_robot*correct_axes;
    //    T_table_robot(0,3) = -1.05;
    //    T_table_robot(1,3) = -0;
    //    T_table_robot(2,3) = -0.37;
    //    Eigen::Matrix3f m;
    //    m = Eigen::AngleAxisf(0, Eigen::Vector3f::UnitX())
    //            * Eigen::AngleAxisf(0,  Eigen::Vector3f::UnitY())
    //            * Eigen::AngleAxisf(0, Eigen::Vector3f::UnitZ());
    //    T_table_robot.block(0,0,3,3) = m;
    //    T_table_robot(3,0) = 0;T_table_robot(3,1) = 0;T_table_robot(3,2) = 0;T_table_robot(3,3) = 1;
    //    T_kinect_robot = T_kinect_robot*T_table_robot;
    //    T_kinect_robot = T_kinect_robot.inverse().eval();
}

void ProjectorKinectCalibration::timerEvent(QTimerEvent* timerEvent)
{
    ros::spinOnce();
    usleep(100);
}

void ProjectorKinectCalibration::saveCalibration(std::string filename)
{
    std::fstream writer;
    writer.open(filename.c_str(), std::ios_base::out);
    writer << "[PROJECTION_MATRIX]" << "\n";
    writer << "P_00=" << projection_matrix(0,0) << "\n";
    writer << "P_01=" << projection_matrix(0,1) << "\n";
    writer << "P_02=" << projection_matrix(0,2) << "\n";
    writer << "P_03=" << projection_matrix(0,3) << "\n";
    writer << "P_10=" << projection_matrix(1,0) << "\n";
    writer << "P_11=" << projection_matrix(1,1) << "\n";
    writer << "P_12=" << projection_matrix(1,2) << "\n";
    writer << "P_13=" << projection_matrix(1,3) << "\n";
    writer << "P_20=" << projection_matrix(2,0) << "\n";
    writer << "P_21=" << projection_matrix(2,1) << "\n";
    writer << "P_22=" << projection_matrix(2,2) << "\n";
    writer << "P_23=" << projection_matrix(2,3) << "\n";
    writer.close();
}

void ProjectorKinectCalibration::normalizeObjectPoints( vector< Eigen::Vector3d > toBeNormalizedOntoPlane){    //find a plane to fit all the points
    //and then replace the points by projecting them onto the plane

    //    algorithm trial:
    //        3x + 2.6y - z + 7.8 = 0
    //    calibration.objectPoints.clear();
    //    Eigen::Vector3d tmp;
    //    tmp << 1, 1, 13.4;
    //    calibration.objectPoints.push_back(tmp);
    //    tmp << 2, 1, 16.4;
    //    calibration.objectPoints.push_back(tmp);
    //    tmp << -1, -3, -3;
    //    calibration.objectPoints.push_back(tmp);

    int size = toBeNormalizedOntoPlane.size();
    if(size){
        //find plane
        Eigen::Vector3d normal;
        Eigen::Vector3d point;


        //if using ransac
        if(false){
            std::vector<int> myvector;

            // set some values:
            for (int i=0; i<size; ++i) myvector.push_back(i); // 1 2 3 4 5 6 7 8 9

            Eigen::Vector3d normal_candidate, point_candidate;
            int best_inlier_count = 0;
            double threshold = 0.01;

            int N = 10;
            for(int i = 0; i < N; ++i){//find the best cadidate out of N minimum set
                // using built-in random generator:
                std::random_shuffle ( myvector.begin(), myvector.end() );

                normal_candidate = (toBeNormalizedOntoPlane[myvector[0]]
                                    -toBeNormalizedOntoPlane[myvector[2]]).cross(toBeNormalizedOntoPlane[myvector[1]]-toBeNormalizedOntoPlane[myvector[2]]);
                point_candidate = toBeNormalizedOntoPlane[myvector[2]];

                int inlier_count = 0;
                for(int p = 0; p < size; ++p){
                    if((toBeNormalizedOntoPlane[p]-toBeNormalizedOntoPlane[myvector[2]]).dot(normal_candidate) < threshold)
                        inlier_count++;
                }
                if(inlier_count > best_inlier_count){
                    best_inlier_count = inlier_count;
                    point = point_candidate;
                    normal = normal_candidate;
                }
            }
            std::cout << "Best inlier number: " << best_inlier_count << std::endl;

            //get all the inliers
            vector< Eigen::Vector3d > inlierPoints;
            for(int p = 0; p < size; ++p){
                if((toBeNormalizedOntoPlane[p]-point).dot(normal) < threshold)
                    inlierPoints.push_back(toBeNormalizedOntoPlane[p]);
            }

            //reestimate normal & plane point
            float s_x = 0, s_y =0, s_z = 0, s_xx = 0, s_xy =0, s_yy = 0, s_xz = 0, s_yz = 0;
            for(int i = 0; i < size; i++){
                s_xx += inlierPoints[i](0) * inlierPoints[i](0);
                s_xy += inlierPoints[i](0) * inlierPoints[i](1);
                s_yy += inlierPoints[i](1) * inlierPoints[i](1);
                s_xz += inlierPoints[i](0) * inlierPoints[i](2);
                s_yz += inlierPoints[i](1) * inlierPoints[i](2);
                s_x += inlierPoints[i](0);
                s_y += inlierPoints[i](1);
                s_z += inlierPoints[i](2);
            }
            Eigen::Matrix3d A;
            A <<    s_xx, s_xy, s_x,
                    s_xy, s_yy, s_y,
                    s_x,  s_y,  size;

            Eigen::Vector3d b;
            b << s_xz, s_yz, s_z;

            Eigen::Vector3d x;
            x = A.colPivHouseholderQr().solve(b);

            normal << x(0), x(1), -1;
            normal.normalize();
            point << 1, 1, x(0)+x(1)+x(2);


        }
        //if using all data/least square:
        else{
            float s_x = 0, s_y =0, s_z = 0, s_xx = 0, s_xy =0, s_yy = 0, s_xz = 0, s_yz = 0;
            for(int i = 0; i < size; i++){
                s_xx += toBeNormalizedOntoPlane[i](0) * toBeNormalizedOntoPlane[i](0);
                s_xy += toBeNormalizedOntoPlane[i](0) * toBeNormalizedOntoPlane[i](1);
                s_yy += toBeNormalizedOntoPlane[i](1) * toBeNormalizedOntoPlane[i](1);
                s_xz += toBeNormalizedOntoPlane[i](0) * toBeNormalizedOntoPlane[i](2);
                s_yz += toBeNormalizedOntoPlane[i](1) * toBeNormalizedOntoPlane[i](2);
                s_x += toBeNormalizedOntoPlane[i](0);
                s_y += toBeNormalizedOntoPlane[i](1);
                s_z += toBeNormalizedOntoPlane[i](2);
            }
            Eigen::Matrix3d A;
            A <<    s_xx, s_xy, s_x,
                    s_xy, s_yy, s_y,
                    s_x,  s_y,  size;

            Eigen::Vector3d b;
            b << s_xz, s_yz, s_z;

            Eigen::Vector3d x;
            x = A.colPivHouseholderQr().solve(b);

            std::cout << "abc: " << x << std::endl;

            normal << x(0), x(1), -1;
            std::cout << "n: " << normal << std::endl;
            normal.normalize();
            point << 0, 0, x(2);
            std::cout << "n: " << normal << std::endl;
            std::cout << "p: " << point << std::endl;

        }


        //project origional points to the plane
        for(int i = 0; i < size; i++){
            //std::cout << i << std::endl;
            //std::cout << toBeNormalizedOntoPlane[i](0) << " " << toBeNormalizedOntoPlane[i](1) << " " << toBeNormalizedOntoPlane[i](2) << std::endl;
            calibration.objectPoints.push_back(toBeNormalizedOntoPlane[i] - normal.dot(toBeNormalizedOntoPlane[i]-point)*normal);
            //std::cout << toBeNormalizedOntoPlane[i](0) << " " << toBeNormalizedOntoPlane[i](1) << " " << toBeNormalizedOntoPlane[i](2) << std::endl;

        }
        std::cout << "done " << std::endl;
    }
    else
        return;

}

void ProjectorKinectCalibration::image_callback(const sensor_msgs::ImageConstPtr& msg)
{
    //std::cout << "Image callback runs!" << std::endl;
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    cv::Mat temp; // make the same cv::Mat
    cv::cvtColor(cv_ptr->image, temp, CV_BGR2RGB);
    cv::Size boardSize;
    boardSize.width = mWidget->getcaliBoard_X_TOTAL()-1;
    boardSize.height = mWidget->getcaliBoard_Y_TOTAL()-1;

    cv::Mat viewGray;
    cv::cvtColor(temp, viewGray, cv::COLOR_RGB2GRAY);
    std::vector<cv::Point2f> pointbuf;
    bool found = cv::findChessboardCorners( temp, boardSize, pointbuf,
                                            CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE
                                            + CALIB_CB_FAST_CHECK);
    if(!calibration_done)
        std::cout << pointbuf.size() << std::endl;
    if (found) cornerSubPix( viewGray, pointbuf, cv::Size(11,11),
                             cv::Size(-1,-1), cv::TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ));
    if(calibration_done)
    std::cout << pointbuf.size() << std::endl;
    if(pointbuf.size() != boardSize.width*boardSize.height){
        if(!calibration_done){
            ROS_ERROR("Part of the calibration board can not be seen by camera!");
            ROS_ERROR("If you see this message multiple times and no calibration result obtained, please ajust calibration board parameters in 'widget.h' and try again!");
        }
        return;
    }
    //    else
    //        calibration_done = false;
    setPoints2D(pointbuf);
}

void ProjectorKinectCalibration::kinect_callback (const sensor_msgs::PointCloud2ConstPtr& input)
{
    bool singleFrame = false;
    if(singleFrame){
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::fromROSMsg (*input, *cloud);
    std::vector<cv::Point2f> points2d_local;
    getPoints2D(points2d_local);
    if(points2d_local.size() == 0)
        return;
    if(!calibration_done)
    {
        calibration.imagePoints.clear();
        calibration.objectPoints.clear();
        for(int point_id = 0;point_id < points2d_local.size();point_id++)
        {
            Eigen::Vector2d image_point;
            Eigen::Vector3d object_point;
            Eigen::Vector4f object_point_hm;
            image_point << CB_X + (1+point_id%(CB_X_TOTAL-1))*CB_SIZE, CB_Y + (1+point_id/(CB_X_TOTAL-1))*CB_SIZE;
            int x = points2d_local[point_id].x;
            int y = points2d_local[point_id].y;
            object_point_hm << cloud->at(x, y).x, cloud->at(x, y).y, cloud->at(x, y).z, 1;
            object_point_hm = T_kinect_robot*object_point_hm;
            object_point << object_point_hm(0), object_point_hm(1), object_point_hm(2);
            //            std::cout << image_point(0) << "," << image_point(1) << "--" << object_point.transpose() << std::endl;
            calibration.imagePoints.push_back(image_point);
            calibration.objectPoints.push_back(object_point);
        }
        calibration.computeProjectionMatrix();
        projection_matrix = calibration.P;
        calibration_done = true;
        std::cout << "calibration done!" << std::endl;
        std::stringstream projection_matrix_fname;
        projection_matrix_fname << ros::package::getPath("perception_sensor_calibration") << "/src/projector_kinect_calibration.ini";
        std::cout << projection_matrix_fname.str() << std::endl;
        saveCalibration(projection_matrix_fname.str());
    }

    std::vector<QPoint> projected_points;
    for(int point_id = 0;point_id < points2d_local.size();point_id++)
    {
        Eigen::Vector4d object_point;
        Eigen::Vector4f object_point_hm;
        Eigen::Vector3d projected_point;
        int x = points2d_local[point_id].x;
        int y = points2d_local[point_id].y;
        object_point_hm << cloud->at(x, y).x, cloud->at(x, y).y, cloud->at(x, y).z, 1;
        object_point_hm = T_kinect_robot*object_point_hm;
        object_point << object_point_hm(0), object_point_hm(1), object_point_hm(2), 1;
        projected_point = projection_matrix*object_point;
        projected_point(0) /= projected_point(2);
        projected_point(1) /= projected_point(2);
        projected_point(2) = 1;
        QPoint point;
        point.setX(projected_point(0));
        point.setY(projected_point(1));
        projected_points.push_back(point);
        //        std::cout << point.x() << "," << point.y() << std::endl;
    }
    mWidget->setProjectedPoints(projected_points);
    mWidget->animate();
    this->centralWidget()->update();
    }
    else{

//        if(elapsFrame < 10){
//            elapsFrame++;
//            return;
//        }
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::fromROSMsg (*input, *cloud);
        std::vector<cv::Point2f> points2d_local;
        getPoints2D(points2d_local);
        if(points2d_local.size() == 0)
            return;
        if(!calibration_done)
        {
            char c;
            std::cout << "Select current frame?(y/n)" << std::endl;
            std::cin >> c;
//            elapsFrame = 0;
            if(c == 'n' || c == 'N')
                return;
            if(!frameCount){
            calibration.imagePoints.clear();
            calibration.objectPoints.clear();
            }

            vector< Eigen::Vector3d > objectPointsCurrentFrame;
            for(int point_id = 0;point_id < points2d_local.size();point_id++)
            {
                Eigen::Vector2d image_point;
                Eigen::Vector3d object_point;
                Eigen::Vector4f object_point_hm;
                image_point << mWidget->getcaliBoardStart_x() + (1+point_id%(CB_X_TOTAL-1))*CB_SIZE, mWidget->getcaliBoardStart_y() + (1+point_id/(CB_X_TOTAL-1))*CB_SIZE;
                int x = points2d_local[point_id].x;
                int y = points2d_local[point_id].y;
                object_point_hm << cloud->at(x, y).x, cloud->at(x, y).y, cloud->at(x, y).z, 1;
                object_point_hm = T_kinect_robot*object_point_hm;
                object_point << object_point_hm(0), object_point_hm(1), object_point_hm(2);
                //            std::cout << image_point(0) << "," << image_point(1) << "--" << object_point.transpose() << std::endl;
                calibration.imagePoints.push_back(image_point);
                objectPointsCurrentFrame.push_back(object_point);
            }
            normalizeObjectPoints(objectPointsCurrentFrame);


            frameCount++;
            mWidget->setFrameIndex(frameCount);

            std::cout << "3d points size: " << calibration.objectPoints.size() << std::endl;
            std::cout << "2d points size: " << calibration.imagePoints.size() << std::endl;

            if (frameCount == 6){
               calibration.computeProjectionMatrix();
               projection_matrix = calibration.P;
               calibration_done = true;
               //elapsFrame = 10;
               mWidget->setCalibrationDone(true);
               std::cout << "calibration done!" << std::endl;
               std::stringstream projection_matrix_fname;
               projection_matrix_fname << ros::package::getPath("perception_sensor_calibration") << "/src/projector_kinect_calibration.ini";
               std::cout << projection_matrix_fname.str() << std::endl;saveCalibration(projection_matrix_fname.str());
           }
            mWidget->animate();
        }

        else{
        std::vector<QPoint> projected_points;
        for(int point_id = 0;point_id < points2d_local.size();point_id++)
        {
            Eigen::Vector4d object_point;
            Eigen::Vector4f object_point_hm;
            Eigen::Vector3d projected_point;
            int x = points2d_local[point_id].x;
            int y = points2d_local[point_id].y;
            object_point_hm << cloud->at(x, y).x, cloud->at(x, y).y, cloud->at(x, y).z, 1;
            object_point_hm = T_kinect_robot*object_point_hm;
            object_point << object_point_hm(0), object_point_hm(1), object_point_hm(2), 1;
            projected_point = projection_matrix*object_point;
            projected_point(0) /= projected_point(2);
            projected_point(1) /= projected_point(2);
            projected_point(2) = 1;
            QPoint point;
            point.setX(projected_point(0));
            point.setY(projected_point(1));
            projected_points.push_back(point);
            //        std::cout << point.x() << "," << point.y() << std::endl;
        }
        mWidget->setProjectedPoints(projected_points);
        mWidget->animate();
        this->centralWidget()->update();
        }
    }


}

void ProjectorKinectCalibration::loadCalibrationLeap(std::string filename)
{
    QString name(filename.c_str());
    QFileInfo config(name);

    if(!config.exists())
    {
        std::cout<< "error reading calibration file: "<< filename << std::endl;
        throw;
    }

    QSettings iniFile(name, QSettings::IniFormat);

    projection_matrix_leap.resize(3,4);
    iniFile.beginGroup("projection_matrix_leap");
    projection_matrix_leap(0,0) = iniFile.value("P_00", "INVALID").toFloat();
    projection_matrix_leap(0,1) = iniFile.value("P_01", "INVALID").toFloat();
    projection_matrix_leap(0,2) = iniFile.value("P_02", "INVALID").toFloat();
    projection_matrix_leap(0,3) = iniFile.value("P_03", "INVALID").toFloat();
    projection_matrix_leap(1,0) = iniFile.value("P_10", "INVALID").toFloat();
    projection_matrix_leap(1,1) = iniFile.value("P_11", "INVALID").toFloat();
    projection_matrix_leap(1,2) = iniFile.value("P_12", "INVALID").toFloat();
    projection_matrix_leap(1,3) = iniFile.value("P_13", "INVALID").toFloat();
    projection_matrix_leap(2,0) = iniFile.value("P_20", "INVALID").toFloat();
    projection_matrix_leap(2,1) = iniFile.value("P_21", "INVALID").toFloat();
    projection_matrix_leap(2,2) = iniFile.value("P_22", "INVALID").toFloat();
    projection_matrix_leap(2,3) = iniFile.value("P_23", "INVALID").toFloat();
    iniFile.endGroup();
}

void ProjectorKinectCalibration::loadCalibrationKinect(std::string filename)
{
    QString name(filename.c_str());
    QFileInfo config(name);

    if(!config.exists())
    {
        std::cout<< "error reading calibration file: "<< filename << std::endl;
        throw;
    }

    QSettings iniFile(name, QSettings::IniFormat);

    projection_matrix_kinect.resize(3,4);
    iniFile.beginGroup("projection_matrix_kinect");
    projection_matrix_kinect(0,0) = iniFile.value("P_00", "INVALID").toFloat();
    projection_matrix_kinect(0,1) = iniFile.value("P_01", "INVALID").toFloat();
    projection_matrix_kinect(0,2) = iniFile.value("P_02", "INVALID").toFloat();
    projection_matrix_kinect(0,3) = iniFile.value("P_03", "INVALID").toFloat();
    projection_matrix_kinect(1,0) = iniFile.value("P_10", "INVALID").toFloat();
    projection_matrix_kinect(1,1) = iniFile.value("P_11", "INVALID").toFloat();
    projection_matrix_kinect(1,2) = iniFile.value("P_12", "INVALID").toFloat();
    projection_matrix_kinect(1,3) = iniFile.value("P_13", "INVALID").toFloat();
    projection_matrix_kinect(2,0) = iniFile.value("P_20", "INVALID").toFloat();
    projection_matrix_kinect(2,1) = iniFile.value("P_21", "INVALID").toFloat();
    projection_matrix_kinect(2,2) = iniFile.value("P_22", "INVALID").toFloat();
    projection_matrix_kinect(2,3) = iniFile.value("P_23", "INVALID").toFloat();
    iniFile.endGroup();
}

ProjectorKinectCalibration::~ProjectorKinectCalibration()
{
    delete ui;
}

void ProjectorKinectCalibration::getPoints2D(std::vector<cv::Point2f> &points_2d)
{
    mutex_points_2d.lock();
    points_2d = points2d;
    mutex_points_2d.unlock();
}

void ProjectorKinectCalibration::setPoints2D(std::vector<cv::Point2f> &points_2d)
{
    mutex_points_2d.lock();
    points2d = points_2d;
    //std::cout << "2d points updated!" << std::endl;
    mutex_points_2d.unlock();
}


