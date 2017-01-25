#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

		void startSlalom();
    void start();		//function for calculation in start state
    void first_pole();	//function for calculation in first pole state
    void second_pole();	//function for calculation in second pole state
    void third_pole();	//function for calculation in third pole state
    void fourth_pole();	//function for calculation in fourth pole state
    int setGoal(double x, double y, double w, double z);		//function for transmitting a goal position to move_base
    
    //declaration of variables
		enum State {START, FIRST_POLE, SECOND_POLE, THIRD_POLE, FOURTH_POLE};
		State current_state;
		int result;
		double goal_x;
    double goal_y;
    double goal_yaw;
		


int main(int argc, char** argv){
  ros::init(argc, argv, "slalom");

  //tell the action client that we want to spin a thread by default


	//create node handle
	ros::NodeHandle nh;
	
	startSlalom();
	
	/*
  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }
	*/

	/*
  move_base_msgs::MoveBaseGoal goal;


  //we'll send a goal to the robot to move 1 meter forward
  goal.target_pose.header.frame_id = "base_link";
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = 1.0;
  goal.target_pose.pose.orientation.w = 1.0;

  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Hooray, the base moved 1 meter forward");
  else
    ROS_INFO("The base failed to move forward 1 meter for some reason");
	*/
  return 0;
}

void startSlalom(){
	ros::Rate loop_rate(5);
	current_state = START;
	while(ros::ok()){
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

int setGoal(double x, double y, double yaw)
{
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

void start()
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

void first_pole()
{
    goal_x = 1.5; // in m 
    goal_y = -1.1;
	goal_yaw = +0.785; // -45 deg. 
    
    //TODO: Calculate!
    
    result = setGoal(goal_x, goal_y, goal_yaw);
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

void second_pole()
{
    goal_x = 1.5; // in m 
    goal_y = 1.1;
		goal_yaw = -0.785; // -45 deg. 
    
    //TODO: Calculate!
    
    result = setGoal(goal_x, goal_y, goal_yaw);
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

void third_pole()
{
    goal_x = 1.5; // in m 
    goal_y = -1.1;
		goal_yaw = +0.785; // -45 deg. 
    
    //TODO: Calculate!
    
    result = setGoal(goal_x, goal_y, goal_yaw);
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

void fourth_pole()
{
    goal_x = 1.5; // in m 
    goal_y = 0.55;
		goal_yaw = -0.785; // -45 deg. 
    
    //TODO: Calculate!
    
    result = setGoal(goal_x, goal_y, goal_yaw);
    if(result == 1)
    {
		ROS_INFO("The base moved to the end! AWESOME!");
	}
	else
	{	
		ROS_INFO("There has been an error in fourth pole state!");
	}
    
}






