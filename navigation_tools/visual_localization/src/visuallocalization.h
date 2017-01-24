#ifndef VISUALLOCALIZATION_H
#define VISUALLOCALIZATION_H
#include <ros/ros.h>
#include <iostream>
#include <cmath>
#include <string>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <cv_bridge/cv_bridge.h>

#include "my_types.h"
#include "landmarkmatcher.h"

using namespace Eigen;
using namespace std;
using namespace cv;

namespace visionSettings {
const double cam_fov = 50; // TO MEASURE
const double laser_range = 240;
const double image_width = 640;
const double image_height = 480;
const double pixelToCamResolution = cam_fov/image_width; /// pxl->deg in cam fov
const double camToLaserResolution = laser_range/cam_fov; /// deg in cam fov -> deg in lidar
}


class VisualLocalization
{
public:
    static constexpr double min_scan_angle = -0.57 + M_PI; ///< remove M_PI for gazebo
    static constexpr double max_scan_angle = 0.57 + M_PI;
    static constexpr double world_distance_btwn_tpl = 2.0; // HAVE TO MEASURE !!
    VisualLocalization(ros::NodeHandle _nh, vector<string> &_lm_names);

private:
    double convertPointToAngle(TemplateData& _pt);
    void projectPixelToCamera(Vector3f _pxlHom);
    void projectCameraToOrigin();
    void transformQuaternionToR();
    void broadcastCameraFrame();
    void localizeCar();
    void locateLandmarksInMap();
    void cbTplDetect(const geometry_msgs::PoseArray::ConstPtr &msg);
    void cbMap(const sensor_msgs::ImageConstPtr& msg);
    void cbLaserScan(const sensor_msgs::LaserScan::ConstPtr &scan);

    // intrinsics
    Matrix3f m_K;
    // extrinsics
    Matrix3f m_R;
    Vector3f m_T;

    CarPosition m_carPosition;
    vector<TemplateData> m_detectedTpl;
    Vector3f m_pixelHom;
    VectorXf m_worldHom;
    // tf stuff
    tf::Vector3 m_camPositionInBaseLink;
    tf::TransformListener m_tfListener;
    // ros
    ros::NodeHandle &m_nh;
    ros::Subscriber m_subTplDetection;
    ros::Subscriber m_subMap;

    cv::Mat m_mapImg;
    bool m_gotMap;

    vector<LandmarkData> m_landmarks;
    LandmarkMatcher * m_pLandmarkMatcher;

};


#endif // VISUALLOCALIZATION_H