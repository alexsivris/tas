#include <iostream>
#include <ros/ros.h>
#include "src/landmarkdetector.h"
int main(int argc, char** argv){
    ros::init(argc, argv, "landmarkdetector_node");
    ros::NodeHandle nh;

    LandmarkDetector nd;

    vector<String> files { "/home/benni/catkin_ws/devel/lib/object_detection/biere.jpg" , "/home/benni/catkin_ws/devel/lib/object_detection/tum.png" , "/home/benni/catkin_ws/devel/lib/object_detection/zaptros.png" };
    nd.computeTemplates(files);
    nd.detectLandmarks(files);

    return 0;
}

