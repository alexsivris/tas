#ifndef SLALOM_H
#define SLALOM_H

#include <math.h>

//ROS headers
#include <iostream>
#include <unistd.h>
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/LaserScan.h>

const static double PI = 3.14159265358979323; // definitino of PI value


class Slalom {

	public:

		Slalom();
		// initialize node handle, the initialization function, return a bool value
		bool init(ros::NodeHandle &nh);
        typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

		//periodically called inside of SLALOM_PACKAGE -> publishers are inside of there
		void update(const ros::Time& time, const ros::Duration& period);

        //declaration of functions
        void startSlalom();
        void start();		//function for calculation in start state
        void first_pole();	//function for calculation in first pole state
        void second_pole();	//function for calculation in second pole state
        void third_pole();	//function for calculation in third pole state
        void fourth_pole();	//function for calculation in fourth pole state
        int setGoal(double x, double y, double w, double z);		//function for transmitting a goal position to move_base
        void scanCallback(sensor_msgs::LaserScan scan_input);
        int setGoal(double x, double y, double yaw);


	private:
		//the ROS node handle
        ros::NodeHandle nh_;
        ros::Subscriber laserSub;

		ros::Time safety_ton_scan_, safety_ton_odom_, safety_ton_mapping_, safety_ton_recognition_ ;

        //declaration of variables
        enum State {START, FIRST_POLE, SECOND_POLE, THIRD_POLE, FOURTH_POLE};
        State current_state;
        int result;
        double goal_x;
        double goal_y;
        double goal_yaw;

        // for laser scan sub
        sensor_msgs::LaserScan scan;
        double scan_angle_increment;
};


#endif // SLALOM_H
