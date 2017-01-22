#include <iostream>
#include <ros/ros.h>
#include "src/landmarkdetector.h"
int main(int argc, char** argv){
    ros::init(argc, argv, "landmarkdetector_node");
    ros::NodeHandle nh;


    cout << "Test" << endl;
    LandmarkDetector nd;

    vector<String> files { "/home/benni/catkin_ws/devel/lib/object_detection/biere.jpg" , "/home/benni/catkin_ws/devel/lib/object_detection/java.jpg" };
    nd.computeTemplates(files);
    nd.detectLandmarks();

    return 0;
}

