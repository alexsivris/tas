#include "send_nav_goals.h"

NavGoals::NavGoals(MoveBaseClient &_ac, ros::NodeHandle & _nh, string &_filename, string &_frameid) :
    ac(_ac), m_nh(_nh), m_fileName(_filename), m_frameId(_frameid)
{
    m_pubWaypoints = m_nh.advertise<geometry_msgs::PoseArray>("tas_nav_goals",15);
    m_pubMarkers = m_nh.advertise<visualization_msgs::Marker>("waypoint_markers",1);
    //some init
    m_xml = new XMLPoses(m_fileName, m_frameId, m_nh);
    m_xml->getWaypoints(m_poseArray, waypoints);
    ROS_INFO("%s", m_frameId.c_str());
}

void NavGoals::startGoalsProcess()
{
    ros::Rate loop_rate(5);
    while(ros::ok())
    {
        // SEND GOALS HERE
        prepareMarkers();
        publishWaypoints();
        sendGoals();
        ros::spinOnce();
        loop_rate.sleep();
    }
}
// markers for waypoints
void NavGoals::prepareMarkers()
{
    m_marker.header.frame_id = m_frameId;
    uint32_t shape = visualization_msgs::Marker::TEXT_VIEW_FACING;
    for (auto i=0;i<waypoints.size(); i++)
    {
        m_marker.header.stamp = ros::Time::now();
        m_marker.ns = "wp_markers";
        m_marker.id = i;
        m_marker.type = shape;
        m_marker.action = visualization_msgs::Marker::ADD;
        m_marker.pose = waypoints.at(i);
        stringstream conv;
        conv << i+1;
        m_marker.text = conv.str();
        m_marker.scale.x = 1.0;
        m_marker.scale.y = 1.0;
        m_marker.scale.z = 1.0;

        m_marker.color.r = 0.0f;
        m_marker.color.g = 1.0f;
        m_marker.color.b = 0.0f;
        m_marker.color.a = 1.0;

        m_marker.lifetime = ros::Duration(); // never auto-delete
        m_pubMarkers.publish(m_marker);

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
    publishWaypoints();
    while (!ac.waitForServer(ros::Duration(5.0))) {
        ROS_INFO("Waiting for the move_base action server to come up");
    }

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

