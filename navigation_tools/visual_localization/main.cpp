#include <iostream>
#include <ros/ros.h>
#include "src/visuallocalization.h"
int main(int argc, char** argv){
    ros::init(argc, argv, "visual_localization_node");
    ros::NodeHandle nh;
    vector<string> lm_names(2);
    lm_names.at(0) = "/home/alex/TAS/recorded_maps/detect/tpl1.png";
    lm_names.at(1) = "/home/alex/TAS/recorded_maps/detect/tpl2.png";

    VisualLocalization vl(nh, lm_names);

    return 0;
}

