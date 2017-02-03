#include "waypointsetter.h"

/**
 * @brief WaypointSetter::WaypointSetter prepare subscriber (initialpose) and publisher (waypoints_set)
 * @param _nh node handle
 */
WaypointSetter::WaypointSetter(ros::NodeHandle &_nh) :
    m_nh(_nh), m_frameId("nav_origin")
{
    m_subPose = m_nh.subscribe("/initialpose",1,&WaypointSetter::cbPose,this);
    m_path = __FILE__;
    m_path.erase(m_path.end()-22,m_path.end());


    string filename;
    filename = m_path + "poses.xml";
    m_theFile.open(filename.c_str());
    m_theFile << "<?xml version=\"1.0\" encoding=\"utf-8\"?>" << std::endl;
    cout << "subscribed! Path: " << filename << endl;

    m_pubWp = m_nh.advertise<geometry_msgs::PoseArray>("waypoints_set",20);
    m_pubMarker = m_nh.advertise<visualization_msgs::Marker>("waypoint_markers",20);
    m_pA.header.frame_id = m_frameId;
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
    savePosesToXml();
}

/**
 * @brief WaypointSetter::cbPose Callback function executed when an initialpose is set
 * @param msg
 */
void WaypointSetter::cbPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
    m_pA.poses.push_back(msg->pose.pose);
    cout << m_pA.poses.back() << endl;
    prepareMarker();
    m_pubWp.publish(m_pA);

}

/**
 * @brief WaypointSetter::prepareMarker prepare style of markers and publish them for visualization.
 */
void WaypointSetter::prepareMarker()
{
    m_marker.header.frame_id = m_frameId;
    uint32_t shape = visualization_msgs::Marker::TEXT_VIEW_FACING;
    for (auto i=0;i<m_pA.poses.size(); i++)
    {
        m_marker.header.stamp = ros::Time::now();
        m_marker.ns = "wp_markers";
        m_marker.id = i;
        m_marker.type = shape;
        m_marker.action = visualization_msgs::Marker::ADD;
        m_marker.pose = m_pA.poses.at(i);

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
        m_pubMarker.publish(m_marker);

    }
}

/**
 * @brief WaypointSetter::savePosesToXml save all the poses to an XML file.
 */
void WaypointSetter::savePosesToXml()
{


    m_theFile << "<poses>" << endl;
    for (auto i=0;i<m_pA.poses.size(); i++)
    {
        //create point nodes
        xml_document<> doc;
        //xml_node<>* root = doc.allocate_node(node_element, "poses");

        xml_node<>* child_point = doc.allocate_node(node_element, "point");
        doc.append_node(child_point);

        stringstream conv;
        conv << i+1;
        string point_name="point_";
        point_name.append(conv.str());

        char * idxStr = doc.allocate_string(point_name.c_str());
        child_point->append_attribute(doc.allocate_attribute("name",idxStr));

        //child coordinates
        xml_node<>* child_point_coordinates = doc.allocate_node(node_element, "coordinates");
        //root->append_node(child_point);
        child_point->append_node(child_point_coordinates);

        xml_node<>* child_point_orientation = doc.allocate_node(node_element, "orientation");
        child_point->append_node(child_point_orientation);
        // orientation
        {
            stringstream convx,convy,convz, convw;
            convx << m_pA.poses.at(i).orientation.x;
            string pos = "<x>" + convx.str() + "</x>";
            convy << m_pA.poses.at(i).orientation.y;
            string pos2 = "<y>" + convy.str() + "</y>";
            pos.append(pos2);
            convz << m_pA.poses.at(i).orientation.z;
            pos2 = "<z>" + convz.str() + "</z>";
            pos.append(pos2);
            convw << m_pA.poses.at(i).orientation.w;
            pos2 = "<w>" + convw.str() + "</w>";
            pos.append(pos2);

            xml_document<> xmlsego;
            xmlsego.parse<0>(&pos[0]);

            xml_node<>* x_node = xmlsego.first_node("x");
            xml_node<> *nodex = doc.clone_node( x_node );
            doc.first_node()->first_node("orientation")->append_node(nodex);
            xml_node<>* y_node = xmlsego.first_node("y");
            xml_node<> *nodey = doc.clone_node( y_node );
            doc.first_node()->first_node("orientation")->append_node(nodey);
            xml_node<>* z_node = xmlsego.first_node("z");
            xml_node<> *nodez = doc.clone_node( z_node );
            doc.first_node()->first_node("orientation")->append_node(nodez);
            xml_node<>* w_node = xmlsego.first_node("w");
            xml_node<> *nodew = doc.clone_node( w_node );
            doc.first_node()->first_node("orientation")->append_node(nodew);
            xmlsego.clear();

        }

        // position
        {
            stringstream convx,convy;
            convx << m_pA.poses.at(i).position.x;
            string pos = "<x>" + convx.str() + "</x>";
            convy << m_pA.poses.at(i).position.y;
            string pos2 = "<y>" + convy.str() + "</y><z>0</z>";
            pos.append(pos2);
            xml_document<> xmlsegx;
            xmlsegx.parse<0>(&pos[0]);

            xml_node<>* x_node = xmlsegx.first_node("x");
            xml_node<> *nodex = doc.clone_node( x_node );
            doc.first_node()->first_node("coordinates")->append_node(nodex);
            xml_node<>* y_node = xmlsegx.first_node("y");
            xml_node<> *nodey = doc.clone_node( y_node );
            doc.first_node()->first_node("coordinates")->append_node(nodey);
            xml_node<>* z_node = xmlsegx.first_node("z");
            xml_node<> *nodez = doc.clone_node( z_node );
            doc.first_node()->first_node("coordinates")->append_node(nodez);
            xmlsegx.clear();

        }
        m_theFile << doc;

        doc.clear();

    }

    m_theFile << "</poses>";

    m_theFile.close();
    //doc.clear();

}
