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


class LandmarkMatcher
{
public:
    LandmarkMatcher(Mat &_src, vector<LandmarkData> &_tpl, ros::NodeHandle &_nh);
private:
    void publishCoordinates(vector<LandmarkData> &_tvec);
    void findLandmarks();

    Mat & m_src;
    vector<LandmarkData> &m_tpl;
    ros::NodeHandle &m_nh;
    ros::Publisher m_lmPub;
};

#endif // LANDMARKMATCHER_H
