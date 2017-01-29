#ifndef MY_TYPES_H
#define MY_TYPES_H
#include <opencv2/opencv.hpp>

#define NO_GOOD_MATCH -5

typedef enum opMode { NO_INIT, INIT } opMode;
enum StarType {
    RED=2,
    BLUE=3,
    GREEN=2
}; //g,b muessen noch bestimmt werden

using namespace cv;
using namespace std;


struct LandmarkData {
    Mat src;
    Point center;
    Point2f map_coordinates;
    unsigned int id=0;
};

struct TemplateImgData {
    double u=0, v=0; /// in pixels [camera]
    unsigned int map_u=0,map_v=0; /// in pixels [map]
    double x=0, y=0; /// in meters
    double theta=0; ///
    double distance=0;
    StarType startype=StarType::RED;
    unsigned int id=0;
};
struct LoadedTemplateData {
    Mat src;
    StarType startype=StarType::RED;
    unsigned int id=0;
};

struct CarPosition {
    double theta=0;
    double u=0, v=0; /// in pixels
    double x=0, y=0; /// in meters
};



#endif // MY_TYPES_H
