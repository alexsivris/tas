#include "send_nav_goals.h"

NavGoals::NavGoals(MoveBaseClient &_ac) :
    ac(_ac)
{
    m_pubWaypoints = m_nh.advertise<geometry_msgs::PoseArray>("tas_nav_goals",15);
    //some init
    initWaypoints();
}
void NavGoals::initWaypoints()
{
    geometry_msgs::Pose waypoint;
    m_poseArray.poses.clear();
    m_poseArray.header.frame_id="map";
    // 1 //
    waypoint.position.x = 4.9;
    waypoint.position.y = -1.7;
    waypoint.position.z = 0.000;
    waypoint.orientation.x = 0.000;
    waypoint.orientation.y = 0.000;
    waypoint.orientation.z = -sqrt(0.5);
    waypoint.orientation.w = sqrt(0.5);
    waypoints.push_back(waypoint);
    m_poseArray.poses.push_back(waypoint);

    // 2 //
    waypoint.position.x = 6.5;
    waypoint.position.y = -6.8;
    waypoint.position.z = 0.000;
    waypoint.orientation.x = 0.000;
    waypoint.orientation.y = 0.000;
    waypoint.orientation.z = 0;
    waypoint.orientation.w = 1;
    waypoints.push_back(waypoint);
    m_poseArray.poses.push_back(waypoint);
    // 3 //
    waypoint.position.x = 10.4;
    waypoint.position.y = -7;
    waypoint.position.z = 0.000;
    waypoint.orientation.x = 0.000;
    waypoint.orientation.y = 0.000;
    waypoint.orientation.z = 0;
    waypoint.orientation.w = 1;
    waypoints.push_back(waypoint);
    m_poseArray.poses.push_back(waypoint);
   
    waypoints.push_back(waypoint);
    m_poseArray.poses.push_back(waypoint);

}

/******/


void NavGoals::startGoalsProcess()
{
    ros::Rate loop_rate(10);
    while(ros::ok())
    {
        // SEND GOALS HERE
        sendGoals();
        ros::spinOnce();
        loop_rate.sleep();
    }
}
// publish waypoints to tas_nav_goals
void NavGoals::publishWaypoints()
{
    m_poseArray.header.stamp = ros::Time::now();

    m_pubWaypoints.publish(m_poseArray);
}

void NavGoals::sendGoals()
{
    while (!ac.waitForServer(ros::Duration(5.0))) { // wait for the action server to come up
        ROS_INFO("Waiting for the move_base action server to come up");
    }
	// just for rviz visualization
        publishWaypoints();
        ROS_INFO("published waypoints!");

    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map"; // set target pose frame of coordinates
    for(int i = 0; i < waypoints.size(); ++i) { // loop over all goal points, point by point
        goal.target_pose.header.stamp = ros::Time::now(); // set current time
        goal.target_pose.pose = waypoints.at(i);
        ROS_INFO("Sending goal");
        ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb); // send goal and register callback handler
        ac.waitForResult(); // wait for goal result

        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO("The base moved to %d goal", i);
        } else {
            ROS_INFO("The base failed to move to %d goal for some reason", i);
        }
    }

}


// static methods
/**
 * Callback function
 */
void NavGoals::doneCb(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result) {
    ROS_INFO("Finished in state [%s]", state.toString().c_str());
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
    /*ROS_INFO("[X]:%f [Y]:%f [W]: %f [Z]: %f", feedback->base_position.pose.position.x,feedback->base_position.pose.position.y,feedback->base_position.pose.orientation.w, feedback->base_position.pose.orientation.z);
*/


}

