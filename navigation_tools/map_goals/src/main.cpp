#include <iostream>
#include <ros/ros.h>
#include "send_nav_goals.h"
int main(int argc, char** argv){
    ros::init(argc, argv, "simple_navigation_goals"); // init and set name
    ros::NodeHandle nh;

    // Parse params
    string xmlFileName, xmlFrameId;
    //nh.param("xml_poses_filename", xmlFileName);
    //nh.param("xml_poses_frameid", xmlFrameId);
xmlFileName = "/home/alex/TAS/catkin_ws/src/tas_group_1/tas_map_goals/poses/poses.xml";
xmlFrameId = "/nav_origin";
    // send navigation goals
    MoveBaseClient ac("move_base", true); // action client to spin a thread by default
    NavGoals ng(ac,nh,xmlFileName, xmlFrameId);
    ng.startGoalsProcess();

    return 0;
}
