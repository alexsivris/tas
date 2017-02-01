#ifndef MY_TYPES_H
#define MY_TYPES_H
#include <opencv2/opencv.hpp>

#define NO_GOOD_MATCH -5

typedef enum opMode { NO_INIT, INIT } opMode;

/**
 * @brief The StarType enum used for description of landmark templates
 */
enum StarType {
    RED=2,
    BLUE=3,
    GREEN=4
};

using namespace cv;
using namespace std;

/**
 * @brief The LandmarkData struct Datatype for landmarks
 */
struct LandmarkData {
    Mat src; ///< landmark image source
    Point center; ///< pixel center of landmark in camera image
    Point2f map_coordinates; ///< map coordinates of pixel center in meters
    unsigned int id=0; ///< unique id
};

/**
 * @brief The TemplateImgData struct Datatype for templates
 */
struct TemplateImgData {
    double u=0, v=0; ///< center in pixels [camera]
    unsigned int map_u=0,map_v=0; ///< center in pixels [map]
    double x=0, y=0; ///< center in meters
    double theta=0; ///< orientation
    double distance=0; ///< distance of template from car
    StarType startype=StarType::RED; ///< type
    unsigned int id=0; ///< unique id
};

/**
 * @brief The LoadedTemplateData struct Datatype for loaded templates
 */
struct LoadedTemplateData {
    Mat src; ///< image source
    StarType startype=StarType::RED; ///< template type
    unsigned int id=0; ///< unique id
};

/**
 * @brief The CarPosition struct Datatype for car position
 */
struct CarPosition {
    double theta=0; ///< orientation
    double u=0, v=0; ///< 2D coordinates in pixels
    double x=0, y=0; ///< 2D coordinates in meters
};



#endif // MY_TYPES_H
