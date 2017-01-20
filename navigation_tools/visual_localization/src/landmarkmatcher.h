#ifndef LANDMARKMATCHER_H
#define LANDMARKMATCHER_H
#include <iostream>
#include <string>
#include <opencv2/opencv.hpp>
#include "my_types.h"

using namespace cv;
using namespace std;


class LandmarkMatcher
{
public:
    LandmarkMatcher(Mat &_src, vector<LandmarkData> &_tpl);
private:
    Mat & m_src;
    vector<LandmarkData> &m_tpl;
    void findLandmarks();
};

#endif // LANDMARKMATCHER_H
