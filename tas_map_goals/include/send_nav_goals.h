#ifndef SEND_NAV_GOALS_H
#define SEND_NAV_GOALS_H

#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseActionResult.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseArray.h>
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
using namespace std;


class NavGoals
{
public:
    NavGoals(MoveBaseClient &_ac);
    //~NavGoals();
    void startGoalsProcess();
private:
    void initWaypoints();
    void sendGoals();
    void publishWaypoints();
    static void doneCb(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result);
    static void activeCb();
    static void feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback);

    vector<geometry_msgs::Pose> waypoints;
    geometry_msgs::PoseArray m_poseArray;
    MoveBaseClient &ac;
    ros::NodeHandle m_nh;
    ros::Publisher m_pubWaypoints;
};

#endif
