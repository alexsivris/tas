#include <iostream>
#include <ros/ros.h>
#include "send_nav_goals.h"

/**
 * @brief main Load XML filename and frame id from parameters in the autonomous_driving.launch
 * file and start NavGoals process.
 * @param argc
 * @param argv
 * @return
 */
int main(int argc, char** argv){
    ros::init(argc, argv, "map_goals");
    ros::NodeHandle nh("~");

    string xmlFileName, xmlFrameId;
    nh.getParam("poses_filename", xmlFileName);
    nh.getParam("poses_frameid", xmlFrameId);
#ifdef DBG
    cout << "XML FILE:" <<  xmlFileName << endl;
    cout << "FRAME ID:" <<  xmlFrameId << endl;
#endif

    MoveBaseClient ac("move_base", true);
    NavGoals ng(ac,nh,xmlFileName, xmlFrameId);
    ng.startGoalsProcess();

    return 0;
}
