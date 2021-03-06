#ifndef LANDMARKDETECTOR_H
#define LANDMARKDETECTOR_H

#ifndef DRAW_MATCHES
#define DRAW_MATCHES
#endif


#include <iostream>
#include <string>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/xfeatures2d.hpp>

#include <iterator>
#include <iostream>

#include "object_detection/landmark.h"

using namespace cv;
using namespace std;
using namespace xfeatures2d;

/**
 * @brief The LandmarkDetector class 
 * Class for detecting landmarks in the camera image using keypoint detectors/descriptors SIFT/SURF/ORB and BruteForceMatcher
 */
class LandmarkDetector
{
public:
    LandmarkDetector();
    void computeTemplates(vector<string> files);
    void detectLandmarks(vector<string> files);
private:
    vector<vector<KeyPoint> > keypoints_vec;
    vector<Mat> descriptors_vec;

    Ptr<FeatureDetector> detector;
    Ptr<DescriptorExtractor> extractor;

    // Parameters
    double ransacReprojThreshold = 20;
};

#endif // LANDMARKDETECTOR_H
