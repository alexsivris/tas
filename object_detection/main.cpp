#include <iostream>
#include <ros/ros.h>
#include "src/landmarkdetector.h"

/**
 * @brief main Load templates from files and start matching them using the camera image. The paths/filenames to the templates are specified in /launch/landmarkdetector.launch
 * @param argc
 * @param argv
 * @return
 */
int main(int argc, char** argv){
    // init ROS node
    ros::init(argc, argv, "landmarkdetector");
    ros::NodeHandle nh("~");

    // load template file paths from launch file
    string tpl_1, tpl_2, tpl_3;
    nh.getParam("template_1",tpl_1);
    nh.getParam("template_2",tpl_2);
    nh.getParam("template_3",tpl_3);

    vector<string> files = {tpl_1, tpl_2, tpl_3};

    // run landmarkdetection
    LandmarkDetector ld;
    ld.computeTemplates(files); // compute template (only once needed)
    ld.detectLandmarks(files); // detect landmarks (runs in a while loop)

    return 0;
}

