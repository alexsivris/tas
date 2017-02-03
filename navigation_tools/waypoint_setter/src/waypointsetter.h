#ifndef WAYPOINTSETTER_H
#define WAYPOINTSETTER_H
#include "ros/ros.h"
#include <iostream>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/Marker.h>
#include <iostream>
#include <fstream>
#include <stdio.h>
#include "rapidxml/rapidxml.hpp"
#include "rapidxml/rapidxml_print.hpp"
#include "rapidxml/rapidxml_utils.hpp"
//#include "rapidxml/rapidxml_iterators.hpp"

using namespace std;
using namespace rapidxml;

/**
 * @brief The WaypointSetter class This tool can be used to conveniently set the waypoints in
 * Rviz. The idea is that all the clicked 2D pose estimates are gathered and automatically saved
 * into an XML file in the end.
 */
class WaypointSetter
{
public:
    WaypointSetter(ros::NodeHandle &_nh);
    void capture();
private:
    void cbPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);
    void prepareMarker();
    void savePosesToXml();

    std::ofstream m_theFile; ///< file handler
    string m_path; ///< path to poses.xml
    const string m_frameId; ///< parent frame id of waypoints
    visualization_msgs::Marker m_marker; ///< gather markers
    ros::Publisher m_pubMarker; ///< publisher of markers (visualization)
    ros::Publisher m_pubWp; ///< publish waypoints as soon as 2D pose estimate is clicked
    ros::Subscriber m_subPose; ///< subscribe to /initialpose
    ros::NodeHandle &m_nh; ///< node handle
    geometry_msgs::PoseArray m_pA; ///< save pose array
};

#endif // WAYPOINTSETTER_H
