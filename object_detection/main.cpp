#include <iostream>
#include <ros/ros.h>
#include "src/landmarkdetector.h"
int main(int argc, char** argv){
    ros::init(argc, argv, "landmarkdetector_node");
    ros::NodeHandle nh;

    LandmarkDetector ld;

    //vector<String> files { "/home/benni/catkin_ws/devel/lib/object_detection/biere.jpg" , "/home/benni/catkin_ws/devel/lib/object_detection/tum.png" , "/home/benni/catkin_ws/devel/lib/object_detection/zaptros.png", "/home/benni/catkin_ws/devel/lib/object_detection/hofbraeuhaus.jpg" };

vector<String> files { "/home/benni/catkin_ws/devel/lib/object_detection/biere.jpg" };


    ld.computeTemplates(files);
    ld.detectLandmarks(files);

    return 0;
}

