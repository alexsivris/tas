#pragma once
#include <ros/ros.h>
#include <string.h>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <stdexcept>
#include "rapidxml/rapidxml.hpp"

#include <geometry_msgs/PoseArray.h>


using namespace std;
using namespace rapidxml;

/**
 * @brief The XMLPoses class In this clas the waypoints are read from an XML file and saved into a vector.
 * The processing of the XML file is done using RapidXML.
 */
class XMLPoses {
public:
    XMLPoses(string &_filename, string &_frameid, ros::NodeHandle &_nh);
    void getWaypoints(geometry_msgs::PoseArray &_posearray, vector<geometry_msgs::Pose> &_waypoints);

private:
    void loadXmlFile();
    void fetchXmlDom();

    geometry_msgs::PoseArray m_poseArray; ///< pose array of waypoints
    vector<geometry_msgs::Pose> m_waypoints; ///< vector of all the waypoints

    string &m_fileName; ///< XML file containing all the poses
    string &m_frameId; ///< frame specifier for pose array
    xml_document<> m_xmlDoc; ///< XML document of poses
    xml_node<> * m_xmlRootNode; ///< pointer to the root node of the xml file
    ros::NodeHandle m_nh; ///< node handle


};
