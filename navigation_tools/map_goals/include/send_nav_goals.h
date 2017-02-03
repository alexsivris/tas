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

/**
 * @brief The NavGoals class In this class the waypoints are sent to the action client.
 * They are also publihed onto the topic "tas_nav_goals" (with markers on "waypoint_markers")
 *  so that they can be visualized in Rviz.
 */
class NavGoals
{
public:
    NavGoals(MoveBaseClient &_ac,ros::NodeHandle &_nh, string _filename, string _frameid, bool _sg);
    ~NavGoals();
    void startGoalsProcess();
private:
    void initWaypoints();
    void sendGoals();
    void publishWaypoints();
    void prepareMarkers();
    static void doneCb(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result);
    static void activeCb();
    static void feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback);

    bool m_sendGoals; ///< flag that is set when the waypoints should be sent to the move base
    string m_fileName; ///< XML file that contains all waypoints
    string m_frameId; ///< frame specifier for waypoints (typically "/nav_origin")
    vector<geometry_msgs::Pose> waypoints; ///< vector of all loaded waypoints
    visualization_msgs::Marker m_marker; ///< marker array for visualization (in rviz)
    geometry_msgs::PoseArray m_poseArray; ///< pose array of waypoints for visualization (in rviz)
    MoveBaseClient &ac; ///< movebase client object for sending the waypoints as goals
    ros::NodeHandle &m_nh; ///< node handle
    ros::Publisher m_pubWaypoints; ///< publisher for waypoints
    ros::Publisher m_pubMarkers; ///< publisher for markers
    XMLPoses *m_xml; /// pointer to XMLPoses class
};

#endif
