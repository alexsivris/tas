#include "send_nav_goals.h"

NavGoals::NavGoals(MoveBaseClient &_ac, ros::NodeHandle & _nh, string &_filename, string &_frameid) :
    ac(_ac), m_nh(_nh), m_fileName(_filename), m_frameId(_frameid)
{
    m_pubWaypoints = m_nh.advertise<geometry_msgs::PoseArray>("tas_nav_goals",15);
    //some init
    m_xml = new XMLPoses(m_fileName, m_frameId, m_nh);
    m_xml->getWaypoints(m_poseArray, waypoints);
}

void NavGoals::startGoalsProcess()
{
    ros::Rate loop_rate(1);
    while(ros::ok())
    {
        // SEND GOALS HERE
        //sendGoals();
        publishWaypoints();
        ROS_INFO("published waypoints!");
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
    while (!ac.waitForServer(ros::Duration(5.0))) {
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    publishWaypoints();
    ROS_INFO("published waypoints!");


    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = m_frameId;
    for(int i = 0; i < waypoints.size(); ++i) {
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.pose = waypoints.at(i);
        ROS_INFO("Sending goal");
        ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
        ac.waitForResult();

        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO("The base moved to %d goal", i);
        } else {
            ROS_INFO("The base failed to move to %d goal for some reason", i);
        }
    }

}

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

