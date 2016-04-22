#ifndef OBJECT_HIGHLIGHTING_H
#define OBJECT_HIGHLIGHTING_H

#include <QMainWindow>
#include <QSettings>
#include <QFileInfo>
#include <QPainter>
#include <QLabel>
#include <QMutex>
#include <fstream>

#include <ros/ros.h>
#include <leap_msgs/Leap.h>
#include <actor_msgs/ActorVec.h>
#include <eigen3/Eigen/Dense>
#include "widget.h"
#include "projectionMatrix.h"

#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv/cv.h>

#include <ros/package.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>


namespace Ui {
class ProjectorKinectCalibration;
}

class ProjectorKinectCalibration : public QMainWindow
{
    Q_OBJECT
    
public:
    explicit ProjectorKinectCalibration(QWidget *parent = 0);
    ~ProjectorKinectCalibration();
    Eigen::MatrixXd projection_matrix;
    Eigen::MatrixXd projection_matrix_leap, projection_matrix_kinect;
    void loadCalibrationLeap(std::string filename);
    void loadCalibrationKinect(std::string filename);
    void loadExtrinsicsKinect(std::string filename);
    void normalizeObjectPoints(vector< Eigen::Vector3d > toBeNormalizedOntoPlane);
    void kinect_callback (const sensor_msgs::PointCloud2ConstPtr& input);
    void image_callback(const sensor_msgs::ImageConstPtr& msg);

    void getPoints2D(std::vector<cv::Point2f> &points_2d);
    void setPoints2D(std::vector<cv::Point2f> &points_2d);
protected:
    virtual void timerEvent(QTimerEvent* timerEvent);

private:
    void saveCalibration(std::string filename);

    Eigen::Matrix4f T_kinect_robot;
    Eigen::Matrix4f T_table_robot;
    ros::Subscriber kinect_subscriber;
    QCursor *cursor;
    Ui::ProjectorKinectCalibration *ui;
    ros::NodeHandle nh;
    image_transport::Subscriber image_sub_;
    ros::Subscriber leap_subscriber;
    ros::Subscriber objects_subscriber;
    Widget* mWidget;
    int timerID; /**< GUI Timer ID */
    std::vector<cv::Point2f> points2d;
    QMutex mutex_points_2d;
    Calib calibration;
    bool calibration_done;
    int frameCount;
    int elapsFrame;

};

#endif // OBJECT_HIGHLIGHTING_H

