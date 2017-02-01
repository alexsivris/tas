#include "slalom.h"

Slalom::Slalom(){};

bool Slalom::init(ros::NodeHandle &nh){
    laserSub = nh.subscribe("base_scan", 1, &Slalom::scanCallback, this);
    current_state = START;
}

void Slalom::update(const ros::Time& time, const ros::Duration& period){
    while(current_state != FOURTH_POLE){
        startSlalom();
    }
}

void Slalom::startSlalom(){
    switch(current_state)
    {
    case START:			start();

    case FIRST_POLE:	first_pole();

    case SECOND_POLE:	second_pole();

    case THIRD_POLE:	third_pole();

    case FOURTH_POLE: 	fourth_pole();
    }
}

int Slalom::setGoal(double x, double y, double yaw){
    MoveBaseClient ac("move_base", true);
    while (!ac.waitForServer(ros::Duration(5.0))) {
        ROS_INFO("Waiting for the move_base action server to come up");
    }
    
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "base_link";
    goal.target_pose.header.stamp = ros::Time::now();
    
    goal.target_pose.pose.position.x = x;
    goal.target_pose.pose.position.y = y;
    goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);

    
    ac.sendGoal(goal);
    ROS_INFO("Sending goal");
    
    ac.waitForResult();
    
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {

        return 1;
    }
    else {
        return 0;
    }
}

void Slalom::start()
{
    goal_x = 1.25; // in m
    goal_y = 0.55;
    goal_yaw = -0.785; // -45 deg.
    
    //TODO: Calculate!
    ROS_INFO("Start() function launched");
    result = setGoal(goal_x, goal_y, goal_yaw);
    if(result == 1)
    {
        ROS_INFO("The base moved to first pole!");
        current_state = FIRST_POLE;
    }
    else
    {
        ROS_INFO("There has been an error in start state!");
    }
    
}

void Slalom::first_pole(){
    double traffic_cone_distance;
    double traffic_cone_angle;
    double scan_angle_increment = 0.36;

    //Search for second traffic cone by using LIDAR
    //Traffic cone must be at the right side of car, so minIndex shall be equal
    //to longitudinal axis of car and maxIndex equal to longitudinal axis + 90°
    int minIndex = ceil((scan.angle_max / scan_angle_increment) / 2);
    int maxIndex = floor((scan.angle_max / scan_angle_increment) / 2 + 90*((scan.angle_max / scan_angle_increment)/240));

    //If the car stands at it's exact goal position, the distance to the traffic
    //cone should be 1.60m
    //Since we want to correct the positioning error of the car we introduce a threshold
    //of 40cm for detecting the traffic cone
    double minDistance = 1.4;
    double maxDistance = 1.8;

    //Now iterate over all sensor data points to find traffic cone
    for (int currIndex = minIndex + 1; currIndex <= maxIndex; currIndex++)
    {
        //search for traffic cone position
        if ((scan.ranges[currIndex] < maxDistance) && (scan.ranges[currIndex] > minDistance))
        {
            traffic_cone_distance = scan.ranges[currIndex];
            traffic_cone_angle = ((maxIndex-minIndex)/90)*currIndex;
        }
    }
    std::cout << "Distance to second traffic cone: " << traffic_cone_distance << std::endl;
    std::cout << "Angle to second traffic cone: " << traffic_cone_angle << std::endl;

    //Calculate next goal position
    goal_x = std::cos(traffic_cone_angle)*traffic_cone_distance;
    goal_y = -(std::sin(traffic_cone_angle)*traffic_cone_distance + 0.55);
    goal_yaw = PI / 4; // 45 deg

    result = setGoal(goal_x, goal_y, goal_yaw);
    if(result == 1)
    {
        ROS_INFO("The base moved to second pole!");
        //go to next state of state_machine
        current_state = SECOND_POLE;
    }
    else
    {
        ROS_INFO("There has been an error in first pole state!");
    }

}




void Slalom::second_pole(){
    double traffic_cone_distance;
    double traffic_cone_angle;
    double scan_angle_increment = 0.36;

    //Search for second traffic cone by using LIDAR
    //Traffic cone must be at the right side of car, so minIndex shall be equal
    //to longitudinal axis of car and maxIndex equal to longitudinal axis + 90°
    int minIndex = ceil((scan.angle_max / scan_angle_increment) / 2);
    int maxIndex = floor((scan.angle_max / scan_angle_increment) / 2 - 90*((scan.angle_max / scan_angle_increment)/240));

    //If the car stands at it's exact goal position, the distance to the traffic
    //cone should be 1.60m
    //Since we want to correct the positioning error of the car we introduce a threshold
    //of 40cm for detecting the traffic cone
    double minDistance = 1.4;
    double maxDistance = 1.8;

    //Now iterate over all sensor data points to find traffic cone
    for (int currIndex = minIndex + 1; currIndex <= maxIndex; currIndex++)
    {
        //search for traffic cone position
        if ((scan.ranges[currIndex] < maxDistance) && (scan.ranges[currIndex] > minDistance))
        {
            traffic_cone_distance = scan.ranges[currIndex];
            traffic_cone_angle = ((maxIndex-minIndex)/90)*currIndex;
        }
    }
    std::cout << "Distance to third traffic cone: " << traffic_cone_distance << std::endl;
    std::cout << "Angle to third traffic cone: " << traffic_cone_angle << std::endl;

    //Calculate next goal position
    goal_x = std::cos(traffic_cone_angle)*traffic_cone_distance;
    goal_y = (std::sin(traffic_cone_angle)*traffic_cone_distance + 0.55);
    goal_yaw = - PI / 4; // 45 deg

    result = setGoal(goal_x, goal_y, goal_yaw);
    if(result == 1)
    {
        ROS_INFO("The base moved to third pole!");
        //go to next state of state_machine
        current_state = THIRD_POLE;
    }
    else
    {
        ROS_INFO("There has been an error in second pole state!");
    }

}

void Slalom::third_pole(){
    double traffic_cone_distance;
    double traffic_cone_angle;
    double scan_angle_increment = 0.36;

    //Search for second traffic cone by using LIDAR
    //Traffic cone must be at the right side of car, so minIndex shall be equal
    //to longitudinal axis of car and maxIndex equal to longitudinal axis + 90°
    int minIndex = ceil((scan.angle_max / scan_angle_increment) / 2);
    int maxIndex = floor((scan.angle_max / scan_angle_increment) / 2 + 90*((scan.angle_max / scan_angle_increment)/240));

    //If the car stands at it's exact goal position, the distance to the traffic
    //cone should be 1.60m
    //Since we want to correct the positioning error of the car we introduce a threshold
    //of 40cm for detecting the traffic cone
    double minDistance = 1.4;
    double maxDistance = 1.8;

    //Now iterate over all sensor data points to find traffic cone
    for (int currIndex = minIndex + 1; currIndex <= maxIndex; currIndex++)
    {
        //search for traffic cone position
        if ((scan.ranges[currIndex] < maxDistance) && (scan.ranges[currIndex] > minDistance))
        {
            traffic_cone_distance = scan.ranges[currIndex];
            traffic_cone_angle = ((maxIndex-minIndex)/90)*currIndex;
        }
    }
    std::cout << "Distance to fourth traffic cone: " << traffic_cone_distance << std::endl;
    std::cout << "Angle to fourth traffic cone: " << traffic_cone_angle << std::endl;

    //Calculate next goal position
    goal_x = std::cos(traffic_cone_angle)*traffic_cone_distance;
    goal_y = -(std::sin(traffic_cone_angle)*traffic_cone_distance + 0.55);
    goal_yaw = PI / 4; // 45 deg

    result = setGoal(goal_x, goal_y, goal_yaw);
    if(result == 1)
    {
        ROS_INFO("The base moved to fourth pole!");
        //go to next state of state_machine
        current_state = FOURTH_POLE;
    }
    else
    {
        ROS_INFO("There has been an error in third pole state!");
    }

}

void Slalom::fourth_pole(){
    ROS_INFO("The car reached the final position. Slalom successfully performed.");
}

void Slalom::scanCallback(sensor_msgs::LaserScan scan_input){
    scan = scan_input;
}
