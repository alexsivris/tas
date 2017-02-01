#include "ros/ros.h"
#include <cmath>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>

using namespace cv;
using namespace std;

struct lmData {
    Mat src;
    Point center;
    Point2f map_coordinates;
};

float g_distanceA; ///< distance from template A
float g_distanceB; ///< distance from template B
float g_distanceC; ///< distance from template C

/**
 * @brief estimatePosition solution of non-linear system of equations is calculated here
 * @param da distance to template A
 * @param db distance to template B
 * @param dc distance to template C
 * @param tpl landmark data containing also the map coordinates
 * @return 2D point of position estimate
 */
Point2f estimatePosition(float da, float db, float dc, vector<lmData> &tpl)
{
    float x_a = tpl.at(0).map_coordinates.x; // 1
    float y_a = tpl.at(0).map_coordinates.y;

    float x_b = tpl.at(1).map_coordinates.x; // 2
    float y_b = tpl.at(1).map_coordinates.y;

    float x_c = tpl.at(2).map_coordinates.x; // 3
    float y_c = tpl.at(2).map_coordinates.y;

    // =====================================

    float d_ab = pow(da,2) - pow(db,2);
    float d_ac = pow(da,2) - pow(dc,2);
    float alpha = pow(x_a,2) - pow(x_b,2);
    float beta = x_b - x_a;
    float gamma = pow(y_a,2) - pow(y_b,2);
    float delta = y_b - y_a;
    float epsilon = pow(x_a,2) - pow(x_c,2);
    float zeta = x_c - x_a;
    float eta = pow(y_a,2) - pow(y_c,2);
    float theta = y_c - y_a;

    // =====================================

    float alpha_prime = d_ab - alpha - gamma;
    float epsilon_prime = d_ac - epsilon - eta;

    // =====================================

    Point2f estimated_position;
    float tmp_num = (beta*epsilon_prime)/zeta;
    estimated_position.x = (-2 * theta * ((alpha_prime - tmp_num)/((2*beta*theta/zeta) - delta)) - epsilon_prime)/(2*zeta);
    float tmp_num2 = (2*beta*theta)/zeta;
    estimated_position.y = (alpha_prime - tmp_num)/(tmp_num2 - delta);
    estimated_position.x = (2 *estimated_position.y*theta + epsilon_prime)/(2*zeta); //(-2 *estimated_position.y*theta - epsilon_prime)/(2*zeta)

    // GOOD
    estimated_position.y = (x_b - x_c)*(pow(x_b,2) - pow(x_a,2) + pow(y_b,2) - pow(y_a,2) + pow(da,2)-pow(db,2));
    estimated_position.y -= (x_a - x_b)*(pow(x_c,2)-pow(x_b,2) + pow(y_c,2) - pow(y_b,2) + pow(db,2)-pow(dc,2));
    estimated_position.y /= 2*((y_a-y_b)*(x_b-x_c) - (y_b-y_c)*(x_a-x_b));

    estimated_position.x = (y_b - y_c)*(pow(y_b,2)-pow(y_a,2) + pow(x_b,2)-pow(x_a,2) + pow(da,2)-pow(db,2));
    estimated_position.x -= (y_a-y_b)*(pow(y_c,2)-pow(y_b,2) + pow(x_c,2)-pow(x_b,2) + pow(db,2)-pow(dc,2));
    estimated_position.x /= 2*((x_a-x_b)*(y_b-y_c) - (x_b-x_c)*(y_a-y_b));
    return -estimated_position;
}


/**
 * @brief publishCoordinates convert pixel to map coordinates and publish landmark position and estimated car position
 * @param _tvec vector of landmark data
 * @param _pub ros publisher
 * @param _pubpose subscriber
 */
void publishCoordinates(vector<lmData> &_tvec, ros::Publisher &_pub, ros::Publisher &_pubpose)
{
    sensor_msgs::PointCloud pcl;
    sensor_msgs::PointCloud pcl2;

    pcl.header.frame_id = "/map";
    pcl.header.stamp = ros::Time::now();
    pcl.header.seq = 0;
    pcl2.header = pcl.header;
    geometry_msgs::Point32 pt32;
    cout << _tvec.at(0).center << endl;
    cout << _tvec.at(1).center << endl;
    for ( int i=0;i<_tvec.size();i++)
    {
        if (i<2)
        {
            pt32.x = 0.05*(_tvec.at(i).center.x) - 100;
            pt32.y =  -0.05 *(_tvec.at(i).center.y) + 100;
            pt32.z = 0;

            _tvec.at(i).map_coordinates.x = pt32.x;
            _tvec.at(i).map_coordinates.y = pt32.y;
        }
        else
        {
            pt32.x = _tvec.at(i).map_coordinates.x;
            pt32.y = _tvec.at(i).map_coordinates.y;
            pt32.z = 0;
        }
        pcl.points.push_back(pt32);
        cout << "Map coord: " << _tvec.at(i).map_coordinates  << endl;
    }
    Point2f est_pos = estimatePosition(g_distanceA, g_distanceB, g_distanceC, _tvec); // 1.8, 2.31, 3.33 | 3.56, 2.83, 1.7 | 1.8, 2.31, 3.33
    pt32.x = est_pos.x;
    pt32.y = est_pos.y;
    pt32.z = 0;
    cout << est_pos << endl << pt32 << endl;

    pcl2.points.push_back(pt32);
    _pub.publish(pcl);
    _pubpose.publish(pcl2);
}

/**
 * @brief main Load two landmark templates from image files and get their center pixels by template matching (the third is actually hardcoded
 * but could as well be obtained from template matching just like the two before).
 * @param argc
 * @param argv
 * @return
 */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "template_matching");
    ros::NodeHandle nh("~");

    ros::Publisher pubTpl = nh.advertise<sensor_msgs::PointCloud>("tpl_in_map",1);
    ros::Publisher pubPose = nh.advertise<sensor_msgs::PointCloud>("pose_in_map",1);

    nh.getParam("distance_from_a",g_distanceA);
    nh.getParam("distance_from_b",g_distanceB);
    nh.getParam("distance_from_c",g_distanceC);
    string impath;
    nh.getParam("map_path",impath);
    Mat img = imread(impath,0);
    Mat imdraw;
    img.copyTo(imdraw);
    Mat img2;
    int match_method = CV_TM_SQDIFF;
    vector<lmData> tplVec(3);
    nh.getParam("template_A", impath);
    tplVec.at(0).src = imread(impath,0);
    nh.getParam("template_B", impath);
    tplVec.at(1).src = imread(impath,0);
    /*tplVec.at(0).src = imread("/home/alex/TAS/recorded_maps/detect/tpl3.png",0);
    tplVec.at(1).src = imread("/home/alex/TAS/recorded_maps/detect/tpl4.png",0);*/

    for (int i=0;i<tplVec.size()-1;i++)
    {
        Mat result;
        result.create(img.rows - tplVec.at(i).src.rows + 1, img.cols - tplVec.at(i).src.cols + 1, CV_32F);

        matchTemplate(img, tplVec.at(i).src, result, match_method);

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

        rectangle( imdraw, matchLoc, Point( matchLoc.x + tplVec.at(i).src.cols , matchLoc.y + tplVec.at(i).src.rows ), Scalar::all(0), 4, 8, 0 );
        tplVec.at(i).center = Point(matchLoc.x + tplVec.at(i).src.cols/2 , matchLoc.y + tplVec.at(i).src.rows/2);

        cout << tplVec.at(i).center << endl;
        circle(imdraw, tplVec.at(0).center, 2, cv::Scalar(255,1,0),4);
        img2 = imdraw(Rect(img.rows/2 - 300,img.rows/2 - 200,850, 800));
    }
#ifdef TRIA_TEST
    tplVec.at(2).map_coordinates.x = 22.4;
    tplVec.at(2).map_coordinates.y = -0.6;
#endif
    namedWindow("Template Matching", WINDOW_NORMAL);
    imshow("Template Matching", img2);
    waitKey(4000);
    ros::Rate rate(4.0);
    while (ros::ok())
    {

        publishCoordinates(tplVec,pubTpl, pubPose);
        rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
