#ifndef LANDMARKDETECTOR_H
#define LANDMARKDETECTOR_H

#include <iostream>
#include <string>
#include <opencv2/opencv.hpp>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include "opencv2/opencv.hpp"

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/xfeatures2d.hpp>

#include <iterator>


using namespace cv;
using namespace std;

class LandmarkDetector
{
public:
    LandmarkDetector();
    void computeTemplates(vector<String> files);
    void detectLandmarks();
private:
    vector<vector<KeyPoint> > keypoints;
    vector<Mat> descriptors_vec;

};

#endif // LANDMARKDETECTOR_H
