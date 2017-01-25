#include "landmarkmatcher.h"

LandmarkMatcher::LandmarkMatcher(Mat &_src, vector<LandmarkData> &_tpl, ros::NodeHandle &_nh) :
    m_src(_src), m_tpl(_tpl), m_nh(_nh)
{
    m_lmPub = _nh.advertise<sensor_msgs::PointCloud>("tpl_in_map",1);
    findLandmarks();
}

/**
 * @brief LandmarkMatcher::publishCoordinates publish world coordinates of landmarks for
 * visualization as a point cloud
 * @param _tvec
 */
void LandmarkMatcher::publishCoordinates(vector<LandmarkData> &_tvec)
{
    sensor_msgs::PointCloud pcl;
    geometry_msgs::Point32 pt32;
    pcl.header.frame_id = "/map";
    pcl.header.stamp = ros::Time::now();
    pcl.header.seq = 0;
#ifdef DBG
    cout << _tvec.at(0).center << endl;
    cout << _tvec.at(1).center << endl;
#endif
    for ( int i=0;i<_tvec.size();i++)
    {
        pt32.x = 0.05*(_tvec.at(i).center.x) - 100;
        pt32.y =  -0.05 *(_tvec.at(i).center.y) + 100;
        pt32.z = 0;
        _tvec.at(i).map_coordinates.x = pt32.x;
        _tvec.at(i).map_coordinates.y = pt32.y;

        pcl.points.push_back(pt32);
#ifdef DBG
        cout << "Map coord: " << _tvec.at(i).map_coordinates  << endl;
#endif
    }
   /* Point2f est_pos = estimatePosition(3.14, 3.92, 4.98, _tvec); // 1.8, 2.31, 3.33 | 3.56, 2.83, 1.7
    pt32.x = est_pos.x;
    pt32.y = est_pos.y;
    pt32.z = 0;
    cout << est_pos << endl << pt32 << endl;*/

    pcl.points.push_back(pt32);
    m_lmPub.publish(pcl);
}


/**
 * @brief LandmarkMatcher::findLandmarks find objects in the map that are associated with the landmarks
 */
void LandmarkMatcher::findLandmarks()
{
    Mat img2;
    Mat imdraw;
    m_src.copyTo(imdraw);
    int match_method = CV_TM_SQDIFF;

    for (int i=0;i<m_tpl.size()-1;i++)
    {
        Mat result;
        result.create(m_src.rows - m_tpl.at(i).src.rows + 1, m_src.cols - m_tpl.at(i).src.cols + 1, CV_32F);

        matchTemplate(m_src, m_tpl.at(i).src, result, match_method);

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

        rectangle( imdraw, matchLoc, Point( matchLoc.x + m_tpl.at(i).src.cols , matchLoc.y + m_tpl.at(i).src.rows ), Scalar::all(0), 4, 8, 0 );
        m_tpl.at(i).center = Point(matchLoc.x + m_tpl.at(i).src.cols/2 , matchLoc.y + m_tpl.at(i).src.rows/2);
        cout << m_tpl.at(i).center << endl;
        circle(imdraw, m_tpl.at(i).center, 2, cv::Scalar(255,1,0),4);
        img2 = imdraw(Rect(m_src.rows/2 - 300,m_src.rows/2 - 200,850, 800));
    }
    publishCoordinates(m_tpl);
}
