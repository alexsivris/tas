#include <iostream>
#include <ros/ros.h>
#include "send_nav_goals.h"
int main(int argc, char** argv){
    ros::init(argc, argv, "simple_navigation_goals"); // init and set name


    MoveBaseClient ac("move_base", true); // action client to spin a thread by default


    // send navigation goals
    NavGoals ng(ac);
    ng.startGoalsProcess();

    return 0;
}
