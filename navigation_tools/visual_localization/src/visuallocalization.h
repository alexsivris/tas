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
 * @brief The VisualLocalization class
 */
class VisualLocalization
{
public:
    static constexpr double min_scan_angle = -1.57; ///< from scan message
    static constexpr double max_scan_angle = 1.57;
    static constexpr double world_distance_btwn_tpl = 2.0; // HAVE TO MEASURE !!
    VisualLocalization(ros::NodeHandle _nh, vector<string> &_lm_names, vector<LoadedTemplateData> &_loaded_tpls);
    void localize();

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
    void cbTplDetect(const geometry_msgs::PoseArray::ConstPtr &msg);
    void cbMap(const sensor_msgs::ImageConstPtr& msg);
    void cbLaserScan(const sensor_msgs::LaserScan::ConstPtr &scan);
    void cbParticles(const geometry_msgs::PoseArray::ConstPtr &_particles);

    geometry_msgs::PoseArray m_particleCloud;

    CarPosition m_carOrigin;

    // tf stuff
    tf::Vector3 m_camPositionInBaseLink; ///< camera frame location relative to base link frame
    tf::TransformListener m_tfListener; ///< tf listener
    tf::TransformBroadcaster m_originBroadcaster;
    // ros
    ros::NodeHandle &m_nh; ///< node handle
    ros::Subscriber m_subScan;
    ros::Subscriber m_subAmclPose; ///< subscribe to pose estimation by amcl
    ros::Subscriber m_subCamImg; ///< subscribe to /usb_cam/image_raw
    ros::Subscriber m_subTplDetection; ///< subscriber to /tpl_detect
    ros::Subscriber m_subMap; ///< subscriber to /map_image/full
    ros::Subscriber m_subParticles; ///< subscriber to /particlecloud
    ros::Publisher m_pubPosition; ///< publisher of pose estimate from localization results
    ros::Publisher m_pubLandmarks;

    geometry_msgs::PoseWithCovariance m_amclPose;
    Mat m_camImg;
    Mat m_mapImg; ///< map in mat format for further processing
    bool m_gotMap; ///< a flag is set when the map image is received
    bool m_gotPoseArray; ///< set true when amcl particle cloud is received
    vector<LoadedTemplateData> &m_loadedTemplates;
    vector<LandmarkData> m_landmarks; ///< contains position of landmarks in the map (world coordinates)
    vector<TemplateImgData> m_detectedTpl; ///< contains position of templates in pixel and world coordinates,as well as the distance from the car
    LandmarkMatcher * m_pLandmarkMatcher; ///< used to find landmarks in map (not in camera image)
    bool m_bInitLocalization;

    LandmarkDetector * m_pLandmarkDetector; /// detect landmarks in camera image

};


#endif // VISUALLOCALIZATION_H
