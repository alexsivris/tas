#ifndef WAYPOINTSETTER_H
#define WAYPOINTSETTER_H
#include "ros/ros.h"
#include <iostream>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseArray.h>
using namespace std;

/**
 * @brief The WaypointSetter class This tool can be used to conveniently set the waypoints in
 * Rviz. The idea is that all the clicked 2D pose estimates are gathered and automatically saved
 * into an XML file in the end. Unfortunately this tool could not be finished.
 */
class WaypointSetter
{
public:
    WaypointSetter(ros::NodeHandle &_nh);
    void capture();
private:
    void cbPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);

    ros::Publisher m_pubWp; ///< publish waypoints as soon as 2D pose estimate is clicked
    ros::Subscriber m_subPose; ///< subscribe to /initialpose
    ros::NodeHandle &m_nh; ///< node handle
    geometry_msgs::PoseArray m_pA; ///< save pose array
};

#endif // WAYPOINTSETTER_H
