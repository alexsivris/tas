#ifndef LANDMARKDETECTOR_H
#define LANDMARKDETECTOR_H
#include <iostream>
#include "my_types.h"
#include <opencv2/opencv.hpp>

/**
 * @brief The LandmarkDetector class In this class the templates of the landmarks are
 * searched in the camera image.
 */
class LandmarkDetector
{
public:
    LandmarkDetector(); // 3 star tpls
    TemplateImgData locate();
    bool foundTemplate(Mat &_img, LoadedTemplateData &_tpl);
    unsigned int getStarId();
private:
    int findBestMatch(vector< vector< Point> > &_img_cnt, vector< Point> &_tpl_strong_cnt, Mat &_tpl_hist);

    vector<vector<cv::Point>> m_tplContours; ///< vector containing contours found in an image
    Mat m_camImg; ///< camera image source
    Mat m_gray; ///< camera image in grayscale
    Mat m_binary; ///< camera image in thresholded binary (values are 0 or 255 only)
    const int m_threshold; ///< threshold for binary image
    Point2f m_center; ///< center of detected template
    float m_radius; ///< radius of ellipse around contour
    Rect m_boundRect; ///< rectangle around contour
    TemplateImgData m_result; ///< result is saved in a struct
};

#endif // LANDMARKDETECTOR_H
