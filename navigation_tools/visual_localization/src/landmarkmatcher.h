#ifndef LANDMARKMATCHER_H
#define LANDMARKMATCHER_H
#include "ros/ros.h"
#include <iostream>
#include <string>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include "my_types.h"
#include <sensor_msgs/PointCloud.h>

using namespace cv;
using namespace std;

/**
 * @brief The LandmarkMatcher class find landmark positions in the map using template matching
 * from OpenCV.
 */
class LandmarkMatcher
{
public:
    LandmarkMatcher(Mat &_src, vector<LandmarkData> &_tpl, ros::Publisher &_pub);
private:
    void publishCoordinates(vector<LandmarkData> &_tvec);
    void findLandmarks();

    Mat & m_src; ///< image source of map
    vector<LandmarkData> &m_tpl; ///< vector of landmark patches used for template matching
    ros::Publisher &m_lmPub; ///< publisher of landmark positions
};

#endif // LANDMARKMATCHER_H
