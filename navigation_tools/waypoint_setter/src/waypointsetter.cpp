#include "waypointsetter.h"

WaypointSetter::WaypointSetter(ros::NodeHandle &_nh) :
    m_nh(_nh)
{
    m_subPose = m_nh.subscribe("initialpose",1,&WaypointSetter::cbPose,this);
    m_pubWp = m_nh.advertise<geometry_msgs::PoseArray>("waypoints_set",1);
    m_pA.header.frame_id = "map";
    m_pA.header.stamp = ros::Time::now();
}

void WaypointSetter::capture()
{
    ros::Rate rt(100);
    while (ros::ok())
    {
        ros::spinOnce();
        rt.sleep();
    }
}

void WaypointSetter::cbPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
    ROS_INFO("RECEIVED!");
    m_pA.poses.push_back(msg->pose.pose);
    m_pubWp.publish(m_pA);

}
