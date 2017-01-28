#ifndef LANDMARKDETECTOR_H
#define LANDMARKDETECTOR_H
#include <iostream>
#include "my_types.h"
#include <opencv2/opencv.hpp>

class LandmarkDetector
{
public:
    LandmarkDetector(); // 3 star tpls
    TemplateImgData locate();
    bool foundTemplate(Mat &_img, LoadedTemplateData &_tpl);
    StarType getStarType();
private:
    int findBestMatch(vector< vector< Point> > &_img_cnt, vector< Point> &_tpl_strong_cnt, Mat &_tpl_hist);
    vector<vector<cv::Point>> m_tplContours;
    Mat m_camImg;
    Mat m_gray;
    Mat m_binary;
    const int m_threshold;
    Point2f m_center;
    float m_radius;
    Rect m_boundRect;

    TemplateImgData m_result;
};

#endif // LANDMARKDETECTOR_H
