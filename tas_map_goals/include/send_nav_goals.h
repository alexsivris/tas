#ifndef SEND_NAV_GOALS_H
#define SEND_NAV_GOALS_H

#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseActionResult.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/Marker.h>
#include "XMLPoses.hpp"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
using namespace std;


class NavGoals
{
public:
    NavGoals(MoveBaseClient &_ac,ros::NodeHandle &_nh, string &_filename, string &_frameid);
    //~NavGoals();
    void startGoalsProcess();
private:
    void initWaypoints();
    void sendGoals();
    void publishWaypoints();
    void prepareMarkers();
    static void doneCb(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result);
    static void activeCb();
    static void feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback);

    string &m_fileName;
    string &m_frameId;
    vector<geometry_msgs::Pose> waypoints;
    visualization_msgs::Marker m_marker;
    geometry_msgs::PoseArray m_poseArray;
    MoveBaseClient &ac;
    ros::NodeHandle &m_nh;
    ros::Publisher m_pubWaypoints;
    ros::Publisher m_pubMarkers;
    XMLPoses *m_xml;
};

#endif
