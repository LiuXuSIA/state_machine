/*************************************************************************
@file           state_machine_demo.cpp
@date           2018/08/20 16:25
@author         liuxu
@email          liuxu.ccc@gmail.com
@description    a simple state machine for the drone race in 2018
*************************************************************************/

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <state_machine/State.h>
#include <state_machine/FIXED_TARGET_POSITION_P2M.h>
#include <state_machine/FIXED_TARGET_RETURN_M2P.h>
#include <state_machine/GRAB_STATUS_M2P.h>
#include <state_machine/TASK_STATUS_CHANGE_P2M.h>
#include <state_machine/TASK_STATUS_MONITOR_M2P.h>
#include <state_machine/VISION_POSITION_GET_M2P.h>
#include <state_machine/YAW_SP_CALCULATED_M2P.h>
#include "math.h"

/**************************for different debug**************************/
#define communication_debug
//#define orientation_debug

/***************************variable definition*************************/

geometry_msgs::PoseStamped pose_pub;  //ENU

/***************************callback function definition***************/
state_machine::State current_state;
void state_cb(const state_machine::State::ConstPtr& msg)
{
    current_state = *msg;
}

geometry_msgs::PoseStamped current_position;
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    current_position = *msg;
}


/*****************************main function*****************************/
int main(int argc, char **argv)
{
    ros::init(argc, argv, "offboard_node");
    ros::NodeHandle nh;
    
    #ifdef orientation_debug
    pose_pub.pose.position.x = 0;
    pose_pub.pose.position.y = 0;
    pose_pub.pose.position.z = 3;
    
    //adjust angular,face north
    float yaw_sp=M_PI_2;
   
    pose_pub.pose.orientation.x = 0;
    pose_pub.pose.orientation.y = 0;
    pose_pub.pose.orientation.z = sin(yaw_sp/2);
    pose_pub.pose.orientation.w = cos(yaw_sp/2);
    #endif

    ros::Subscriber state_sub = nh.subscribe<state_machine::State>("mavros/state",10,state_cb);
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose",10,pose_cb);

    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local",10);
    
    ros::Rate rate(10.0);

    while(ros::ok() && !current_state.connected)
    {
    	ros::spinOnce();
    	rate.sleep();
    }

    ROS_INFO("Connect successfully!!");

    ROS_INFO("send setpoint before takeoff,please wait");

    for(int i =100; ros::ok() && i > 0; i--)
	{
		local_pos_pub.publish(pose_pub);
		ros::spinOnce();
		rate.sleep();
	}

    ROS_INFO("Initialization finished");

	while(ros::ok())
	{
        local_pos_pub.publish(pose_pub);
		ros::spinOnce();
		rate.sleep();
	}
	return 0;
}
