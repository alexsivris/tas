#include "slalom.h"


SlalomGoals::SlalomGoals(MoveBaseClient &_ac, ros::NodeHandle &_nh)
{
	//constructor
	ac = _ac;
	nh = _nh;
}
void SlalomGoals::startSlalom()
{
    ros::Rate loop_rate(5);
    while(ros::ok())
    {
        // Start state machine for different states of slalom parcour
        switch(current_state)
        {
			case START:			start();	
			
			case FIRST_POLE:	first_pole();
			
			case SECOND_POLE:	second_pole();
			
			case THIRD_POLE:	third_pole();
			
			case FOURTH_POLE: 	fourth_pole();
		} 
        
        ros::spinOnce();
        loop_rate.sleep();
    }
}


void SlalomGoals::start()
{
    double goal_x;
    double goal_y;
    double goal_w;
    double goal_z;
    
    //TODO: Calculate!
    
    result = setGoal(goal_x,goal_y,goal_w,goal_z);
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

void SlalomGoals::first_pole()
{
    double goal_x;
    double goal_y;
    double goal_w;
    double goal_z;
    
    //TODO: Calculate!
    
    result = setGoal(goal_x,goal_y,goal_w,goal_z);
    if(result == 1)
    {
		ROS_INFO("The base moved to second pole!");
		current_state = SECOND_POLE;
	}
	else
	{	
		ROS_INFO("There has been an error in first pole state!");
	}
    
}

void SlalomGoals::second_pole()
{
    double goal_x;
    double goal_y;
    double goal_w;
    double goal_z;
    
    //TODO: Calculate!
    
    result = setGoal(goal_x,goal_y,goal_w,goal_z);
    if(result == 1)
    {
		ROS_INFO("The base moved to third pole!");
		current_state = THIRD_POLE;
	}
	else
	{	
		ROS_INFO("There has been an error in second pole state!");
	}
    
}

void SlalomGoals::third_pole()
{
    double goal_x;
    double goal_y;
    double goal_w;
    double goal_z;
    
    //TODO: Calculate!
    
    result = setGoal(goal_x,goal_y,goal_w,goal_z);
    if(result == 1)
    {
		ROS_INFO("The base moved to fourth pole!");
		current_state = FOURTH_POLE;
	}
	else
	{	
		ROS_INFO("There has been an error in third pole state!");
	}
    
}

void SlalomGoals::fourth_pole()
{
    double goal_x;
    double goal_y;
    double goal_w;
    double goal_z;
    
    //TODO: Calculate!
    
    result = setGoal(goal_x,goal_y,goal_w,goal_z);
    if(result == 1)
    {
		ROS_INFO("The base moved to the end! AWESOME!");
	}
	else
	{	
		ROS_INFO("There has been an error in fourth pole state!");
	}
    
}

int SlalomGoals::setGoal(double x, double y, double w, double z)
{
	while (!ac.waitForServer(ros::Duration(5.0))) {
        ROS_INFO("Waiting for the move_base action server to come up");
    }
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "base_link";
    goal.target_pose.header.stamp = ros::Time::now();
    
    goal.target_pose.pose.position.x = x;
    goal.target_pose.pose.position.y = y;
    goal.target_pose.pose.orientation.w = w;
    goal.target_pose.pose.orientation.z = z;
    
    
    
    ROS_INFO("Sending goal");
    ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
    
    ac.waitForResult();
    
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
            
            return 1;
    } 
    else {
            return 0;
    }
}

/**
 * Callback function, called every time a target position is reached
 */
void NavGoals::doneCb(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result) {
    ROS_INFO("Reached target position!");
}
/**
 * Callback function, called once when the goal becomes active
 */
void NavGoals::activeCb() {
    ROS_INFO("Goal just went active");
}
/**
 * Callback function, called every time feedback is received for the goal
 */
void NavGoals::feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback) {
 /**ROS_INFO("[X]:%f [Y]:%f [W]: %f [Z]: %f", feedback->base_position.pose.position.x,feedback->base_position.pose.position.y,feedback->base_position.pose.orientation.w, feedback->base_position.pose.orientation.z);
* NOT USED AT THE MOMENT!
*/
}
