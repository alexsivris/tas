#ifndef SLALOM
#define SLALOM
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseActionResult.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/Marker.h>
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
using namespace std;
class SlalomGoals
{
	MoveBaseClient ac;
	ros::NodeHandle nh;
public:
    SlalomGoals(MoveBaseClient &_ac,ros::NodeHandle &_nh);
    
    //declaration of functions
    void startSlalom();
    void start();		//function for calculation in start state
    void first_pole();	//function for calculation in first pole state
    void second_pole();	//function for calculation in second pole state
    void third_pole();	//function for calculation in third pole state
    void fourth_pole();	//function for calculation in fourth pole state
    int setGoal();		//function for transmitting a goal position to move_base
    
    //declaration of variables
    enum current_state {START, FIRST_POLE, SECOND_POLE, THIRD_POLE, FOURTH_POLE};
    
private:
    
};
#endif
