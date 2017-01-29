#include <iostream>
#include <ros/ros.h>
#include "src/waypointsetter.h"
int main(int argc, char ** argv)
{
	ros::init(argc, argv, "waypoint_setter");
    ros::NodeHandle nh("~");
	ROS_INFO("Subscribing to /initialpose...");
	ROS_INFO("Choose your waypoints as 2D Pose estimates in Rviz.");
	
    WaypointSetter ws(nh);
    ws.capture();

    return 0;
}
