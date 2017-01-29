#ifndef WAYPOINTSETTER_H
#define WAYPOINTSETTER_H
#include "ros/ros.h"
#include <iostream>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseArray.h>
using namespace std;
class WaypointSetter
{
public:
    WaypointSetter(ros::NodeHandle &_nh);
    void capture();
private:
    void cbPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);

    ros::Publisher m_pubWp;
    ros::Subscriber m_subPose;
    ros::NodeHandle &m_nh;
    geometry_msgs::PoseArray m_pA;
};

#endif // WAYPOINTSETTER_H
