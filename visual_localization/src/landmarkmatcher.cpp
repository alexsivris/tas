#include "landmarkmatcher.h"

LandmarkMatcher::LandmarkMatcher(Mat &_src, vector<LandmarkData> &_tpl) :
    m_src(_src), m_tpl(_tpl)
{
    findLandmarks();
}

void LandmarkMatcher::findLandmarks()
{
    Mat img2;
    int match_method = CV_TM_SQDIFF;

    for (auto itpl: m_tpl)
    {
        Mat result;
        result.create(m_src.rows - itpl.src.rows + 1, m_src.cols - itpl.src.cols + 1, CV_32F);

        matchTemplate(m_src, itpl.src, result, match_method);

        normalize( result, result, 0, 1, NORM_MINMAX, -1, Mat() );

        double minVal; double maxVal; Point minLoc; Point maxLoc;
        Point matchLoc;

        minMaxLoc( result, &minVal, &maxVal, &minLoc, &maxLoc, Mat() );

        if( match_method  == CV_TM_SQDIFF || match_method == CV_TM_SQDIFF_NORMED )
        {
            matchLoc = minLoc;
        }
        else
        {
            matchLoc = maxLoc;
        }

        rectangle( m_src, matchLoc, Point( matchLoc.x + itpl.src.cols , matchLoc.y + itpl.src.rows ), Scalar::all(0), 4, 8, 0 );
        itpl.center = Point(matchLoc.x + itpl.src.cols/2 , matchLoc.y + itpl.src.rows/2);
        cout << itpl.center << endl;
        circle(m_src, itpl.center, 2, cv::Scalar(255,1,0),4);
        img2 = m_src(Rect(m_src.rows/2 - 300,m_src.rows/2 - 200,850, 800));
    }
}
