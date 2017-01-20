#ifndef MY_TYPES_H
#define MY_TYPES_H
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

struct LandmarkData {
    Mat src;
    Point center;
};

struct TemplateData {
    double u=0, v=0; /// in pixels [camera]
    unsigned int map_u=0,map_v=0; /// in pixels [map]
    double x=0, y=0; /// in meters
    double theta=0; ///
    double distance=0;
    string desc; /// object description
};

struct CarPosition {
    double u=0, v=0; /// in pixels
    double x=0, y=0; /// in meters
};


#endif // MY_TYPES_H
