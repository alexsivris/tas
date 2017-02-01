#include "XMLPoses.hpp"

/**
 * @brief XMLPoses::XMLPoses In the constructor the XML file is loaded and the necessary member variables
 * are initialized.
 * @param _filename XML file name
 * @param _frameid frame specifier for waypoints
 * @param _nh node handle
 */
XMLPoses::XMLPoses(string &_filename, string &_frameid, ros::NodeHandle &_nh) :
    m_fileName(_filename), m_nh(_nh), m_frameId(_frameid)
{
#ifdef DBG
    cout << "Filename: " <<  m_fileName << endl;
#endif

    try {
        loadXmlFile();
    } catch(string e) {
        ROS_ERROR("%s",e.c_str());
    }

    m_xmlRootNode = m_xmlDoc.first_node("poses");
    fetchXmlDom();

}

/**
 * @brief XMLPoses::getWaypoints assigns the waypoints that have been read out from the XML file to a
 * pose array and a pose vector. These are the "outputs" of this class.
 * @param _posearray reference to pose array
 * @param _waypoints reference to pose vector
 */
void XMLPoses::getWaypoints(geometry_msgs::PoseArray &_posearray, vector<geometry_msgs::Pose> &_waypoints)
{
    _posearray = m_poseArray;
    _waypoints = m_waypoints;

}

/**
 * @brief XMLPoses::fetchXmlDom This is the core method of the class XMLPoses. In this method the XML tree
 * is traversed: the poses are extracted from it and saved into the pose array/vector of poses.
 */
void XMLPoses::fetchXmlDom()
{
    geometry_msgs::Pose waypoint;
    m_poseArray.poses.clear();
    m_poseArray.header.frame_id = m_frameId;

    for (xml_node<> *poses_node = m_xmlRootNode->first_node("point");
         poses_node;
         poses_node = poses_node->next_sibling())
    {
        cout << "Currently looking at: " << poses_node->first_attribute("name")->value() << endl;
        cout << "->Coordinates: " << endl;
        xml_node<> * coord_node = poses_node->first_node("coordinates");
        xml_node<> * x_node = coord_node->first_node("x");
        xml_node<> * y_node = coord_node->first_node("y");
        xml_node<> * z_node = coord_node->first_node("z");
        waypoint.position.x = ::atof(coord_node->first_node("x")->value());
        waypoint.position.y = ::atof(coord_node->first_node("y")->value());
        waypoint.position.z = ::atof(coord_node->first_node("z")->value());

        cout << "-->(f " << ::atof(coord_node->first_node("x")->value()) << ", " << y_node->value() << ", " << z_node->value() << ")" << endl;

        cout << "->Orientation:" << endl;
        xml_node<> * orient_node = poses_node->first_node("orientation");
        waypoint.orientation.x = ::atof(orient_node->first_node("x")->value());
        waypoint.orientation.y = ::atof(orient_node->first_node("y")->value());
        waypoint.orientation.z = ::atof(orient_node->first_node("z")->value());
        waypoint.orientation.w = ::atof(orient_node->first_node("w")->value());
        m_waypoints.push_back(waypoint);
        m_poseArray.poses.push_back(waypoint);

        x_node = orient_node->first_node("x");
        y_node = orient_node->first_node("y");
        z_node = orient_node->first_node("z");
        xml_node<> * w_node = orient_node->first_node("w");
        cout << "-->[f64 " << x_node->value() << ", " << y_node->value() << ", "
             << z_node->value() << ", " << w_node->value() << "]" << endl;
    }
}

/**
 * @brief XMLPoses::loadXmlFile helper method to load the XML file
 */
void XMLPoses::loadXmlFile()
{
    ifstream theFile (m_fileName);
    if (theFile)
    {
        vector<char> vecBuffer((istreambuf_iterator<char>(theFile)), istreambuf_iterator<char>());
        vecBuffer.push_back('\0');
        m_xmlDoc.parse<0>(&vecBuffer[0]);
        ROS_INFO("Successfully loaded %s", m_fileName.c_str());
    }
    else
    {
        throw "XMLPoses: Something went wrong with accessing the XML-file.";
    }
}
