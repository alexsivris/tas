#pragma once
#include <ros/ros.h>
#include <string.h>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include "rapidxml/rapidxml.hpp"

#include <geometry_msgs/PoseArray.h>


using namespace std;
using namespace rapidxml;

class XMLPoses {
public:
    XMLPoses(string &_filename, string &_frameid, ros::NodeHandle &_nh);
    void getWaypoints(geometry_msgs::PoseArray &_posearray, vector<geometry_msgs::Pose> &_waypoints);

private:
    bool loadXmlFile();
    void fetchXmlDom();

    geometry_msgs::PoseArray m_poseArray;
    vector<geometry_msgs::Pose> m_waypoints;

    string &m_fileName;
    string &m_frameId;
    xml_document<> m_xmlDoc;
    xml_node<> * m_xmlRootNode;
    ros::NodeHandle m_nh;


};
