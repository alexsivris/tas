#include <iostream>
#include <ros/ros.h>
#include "src/visuallocalization.h"
int main(int argc, char** argv){
    ros::init(argc, argv, "visual_localization_node");
    ros::NodeHandle nh;
    VisualLocalization vl(nh);

    return 0;
}

