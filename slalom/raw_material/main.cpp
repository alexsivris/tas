#include <iostream>
#include <ros/ros.h>
#include "slalom.h"


int main(int argc, char** argv){
    ros::init(argc, argv, "simple_slalom"); // init and set name
    ros::NodeHandle nh;
    
    // initialise move base and start slalom
    MoveBaseClient ac("move_base", true); // action client to spin a thread by default
    SlalomGoals sg(ac,nh);
    sg.startSlalom();
    
    return 0;
}

