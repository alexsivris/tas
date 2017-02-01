//! VisualLocalization class
#ifndef VISUALLOCALIZATION_H
#define VISUALLOCALIZATION_H
#include <ros/ros.h>
#include <iostream>
#include <cmath>
#include <string>
#include <algorithm>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <sensor_msgs/Image.h>


#include "my_types.h"
#include "landmarkmatcher.h"
#include "landmarkdetector.h"

using namespace std;
using namespace cv;

namespace visionSettings {
const double cam_fov = 64; // TO MEASURE
const double laser_range = 180; /// -1.57 to 1.57 rad => pi or 180 deg.
const double laser_resolution = 720; ///< elements of range vector
const double image_width = 640;
const double image_height = 480;
const double pixelToCamResolution = cam_fov/image_width; /// pxl->deg in cam fov
const double camToLaserResolution = laser_range/cam_fov; /// deg in cam fov -> deg in lidar
}

/**
 * @brief The VisualLocalization class In this class the position of the car is estimated by
 * using visual data from an RGB camera as well as laser data from the Hokuyo laser scanner.
 * The initial position is found by the solution of a nonlinear system of three circle equations and
 * the pose estimate during the driving is done by inspecting the difference between the poses in the particle cloud
 * and the points in the circle around the recognized landmark with radius equal to the distance obtained from the
 * laser scanner (cf. method simpleLocalization(TemplateImgData &_tpl) for more detail).
 */
class VisualLocalization
{
public:
    static constexpr double min_scan_angle = -1.57; ///< from scan message
    static constexpr double max_scan_angle = 1.57;
    static constexpr double world_distance_btwn_tpl = 2.0; // HAVE TO MEASURE !!
    void localize();
    VisualLocalization(ros::NodeHandle _nh, vector<string> &_lm_names, vector<LoadedTemplateData> &_loaded_tpls, bool _use_map_img, string _map_path);
    ~VisualLocalization();
private:
    void publishPose(Point2f &_pose);
    Point2f simpleLocalization(TemplateImgData &_tpl);
    bool anyOfType(vector<TemplateImgData>& _tpl_vec,unsigned int &_id);
    unsigned int getRangeFromAngle(double &_angle);
    double convertPointToAngle(TemplateImgData& _pt);
    void broadcastOriginFrame();
    void calculateOriginOrientation();
    Point2f estimatePosition(vector<LandmarkData> &_lm, float da, float db, float dc);
    void locateLandmarksInMap();
    void cbAmclPose(const geometry_msgs::PoseWithCovariance::ConstPtr &msg);
    void cbCamImg(const sensor_msgs::Image::ConstPtr &_img);
    void cbTplDetect(const geometry_msgs::PoseArray::ConstPtr &msg); // INTERFACE THIS FCT
    void cbMap(const sensor_msgs::ImageConstPtr& msg);
    void cbLaserScan(const sensor_msgs::LaserScan::ConstPtr &scan);
    void cbParticles(const geometry_msgs::PoseArray::ConstPtr &_particles);

    geometry_msgs::PoseArray m_particleCloud;
    CarPosition m_carOrigin; ///< 2D position of car in the map AND orientation
    tf::Vector3 m_camPositionInBaseLink; ///< camera frame location relative to base link frame
    tf::TransformListener m_tfListener; ///< tf listener
    tf::TransformBroadcaster m_originBroadcaster; ///< tf broadcaster for initial position/nav_origin frame
    ros::NodeHandle &m_nh; ///< node handle
    ros::Subscriber m_subScan; ///< subscribe to scan topic
    ros::Subscriber m_subAmclPose; ///< subscribe to pose estimation by amcl
    ros::Subscriber m_subCamImg; ///< subscribe to /usb_cam/image_raw
    ros::Subscriber m_subTplDetection; ///< subscriber to /tpl_detect
    ros::Subscriber m_subMap; ///< subscriber to /map_image/full
    ros::Subscriber m_subParticles; ///< subscriber to /particlecloud
    ros::Publisher m_pubPosition; ///< publisher of pose estimate from localization results
    ros::Publisher m_pubLandmarks; ///< publisher of landmark positions
    geometry_msgs::PoseWithCovariance m_amclPose; ///< AMCL pose estimate (from "/amcl_pose" topic)
    Mat m_camImg; ///< camera image is saved into an OpenCV Mat format
    Mat m_mapImg; ///< map in Mat format for further processing
    bool m_gotMap; ///< a flag is set when the map image is received
    bool m_gotPoseArray; ///< set true when amcl particle cloud is received
    vector<LoadedTemplateData> &m_loadedTemplates; ///< reference to loaded templates
    vector<LandmarkData> m_landmarks; ///< contains position of landmarks in the map (world coordinates)
    vector<TemplateImgData> m_detectedTpl; ///< contains position of templates in pixel and world coordinates,as well as the distance from the car
    LandmarkMatcher * m_pLandmarkMatcher; ///< used to find landmarks in map (not in camera image)
    bool m_bInitLocalization; ///< flag unset when initial localization has successfully happened
    LandmarkDetector * m_pLandmarkDetector; /// detect landmarks in camera image
};


#endif // VISUALLOCALIZATION_H
