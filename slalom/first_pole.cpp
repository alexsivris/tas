void first_pole()
{
	double traffic_cone_distance;
	double traffic_cone_angle;
	
	//Search for second traffic cone by using LIDAR
	//Traffic cone must be at the right side of car, so minIndex shall be equal
	//to longitudinal axis of car and maxIndex equal to longitudinal axis + 90°
	int minIndex = ceil((scan->angle_max / scan_angle_increment) / 2);
	int maxIndex = floor((scan->angle_max / scan_angle_increment) / 2 + 90*((scan->angle_max / scan_angle_increment)/240));
	
	//If the car stands at it's exact goal position, the distance to the traffic
	//cone should be 1.60m
	//Since we want to correct the positioning error of the car we introduce a threshold
	//of 40cm for detecting the traffic cone
	double minDistance = 1.4;
	double maxDistane = 1.8;

	//Now iterate over all sensor data points to find traffic cone
	for (int currIndex = minIndex + 1; currIndex <= maxIndex; currIndex++) 
	{
		//search for traffic cone position
		if ((scan->ranges[currIndex] < maxDistance) && (scan->ranges[currIndex] > minDistance)) 
		{
			traffic_cone_distance = scan->ranges[currIndex];
			traffic_cone_angle = ((maxIndex-minIndex)/90)*currIndex;
			ROS_INFO("Distance to second traffic cone: " << traffic_cone_distance);
			ROS_INFO("Angle to second traffic cone: " << traffic_cone_angle);
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

