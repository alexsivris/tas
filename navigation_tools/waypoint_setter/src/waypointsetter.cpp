#include "waypointsetter.h"

/**
 * @brief WaypointSetter::WaypointSetter prepare subscriber (initialpose) and publisher (waypoints_set)
 * @param _nh node handle
 */
WaypointSetter::WaypointSetter(ros::NodeHandle &_nh) :
    m_nh(_nh)
{
    m_subPose = m_nh.subscribe("initialpose",1,&WaypointSetter::cbPose,this);
    m_pubWp = m_nh.advertise<geometry_msgs::PoseArray>("waypoints_set",1);
    m_pA.header.frame_id = "map";
    m_pA.header.stamp = ros::Time::now();
}

/**
 * @brief WaypointSetter::capture loop until user decides to end the process by pressing Ctrl+C.
 * Then, the poses are saved in an XML file (which hasn't been done) that can be used by the map_goals node
 * for autonomous driving.
 */
void WaypointSetter::capture()
{
    ros::Rate rt(100);
    while (ros::ok())
    {
        ros::spinOnce();
        rt.sleep();
    }
    // TODO: save all poses to xml file
}

/**
 * @brief WaypointSetter::cbPose Callback function executed when an initialpose is set
 * @param msg
 */
void WaypointSetter::cbPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
    ROS_INFO("RECEIVED!");
    m_pA.poses.push_back(msg->pose.pose);
    m_pubWp.publish(m_pA);

}
