/*************************************************************************
@file           state_machine_demo_TL.cpp
@date           2018/09/11 
@author         liuxu
@email          liuxu.ccc@gmail.com
@description    a simple state machine for the drone race in 2018
                takeoff-->hover-->landing
*************************************************************************/

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <state_machine/CommandTOL.h>
#include <state_machine/State.h>
#include "math.h"

/***************************function declare****************************/
void state_machine_fun(void);

/***************************variable definition*************************/

geometry_msgs::PoseStamped pose_pub;  //ENU
geometry_msgs::TwistStamped vel_pub;
ros::Publisher local_pos_pub;
ros::Publisher local_vel_pub;

//state_machine state
static const int takeoff = 1;
static const int target_go = 2;
static const int hover = 3;
static const int land = 4;

//mission 
int current_pos_state = takeoff;

//time
ros::Time last_time;

//land
state_machine::CommandTOL landing_cmd;
ros::ServiceClient land_client;
ros::Time landing_last_request;

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

    vel_pub.twist.linear.x = 0.0f;
    vel_pub.twist.linear.y = 0.0f;
    vel_pub.twist.linear.z = 2.0f;
    vel_pub.twist.angular.x = 0.0f;
    vel_pub.twist.angular.y = 0.0f;
    vel_pub.twist.angular.z = 0.0f;

    pose_pub.pose.position.x = 0;
    pose_pub.pose.position.y = 0;
    pose_pub.pose.position.z = 5;

    ros::Subscriber state_sub = nh.subscribe<state_machine::State>("mavros/state",10,state_cb);
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose",10,pose_cb);
    
    local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local",10);
    local_vel_pub = nh.advertise<geometry_msgs::TwistStamped>("mavros/setpoint_velocity/cmd_vel",10);
    land_client = nh.serviceClient<state_machine::CommandTOL>("mavros/cmd/land");
    landing_cmd.request.min_pitch = 1.0;
    landing_last_request = ros::Time::now();

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
		local_vel_pub.publish(vel_pub);
		ros::spinOnce();
		rate.sleep();
	}

    ROS_INFO("Initialization finished");

	while(ros::ok())
	{
        state_machine_fun();
        ROS_INFO("current_pos_state:%d",current_pos_state);
        
		ros::spinOnce();
		rate.sleep();
	}

	return 0;
}

/***********************************state_machine function***********************************/
void state_machine_fun(void)
{
    switch(current_pos_state)
    {
        case takeoff:
        {
            local_vel_pub.publish(vel_pub);
            if(current_position.pose.position.z > 3)
            {
                current_pos_state = target_go;
                last_time = ros::Time::now();
            }
        }
        break;
        case target_go:
        {
            local_pos_pub.publish(pose_pub);
            if(abs(current_position.pose.position.x - pose_pub.pose.position.x) < 1 &&
               abs(current_position.pose.position.y - pose_pub.pose.position.y) < 1 &&
               abs(current_position.pose.position.z - pose_pub.pose.position.z) < 1 )
            {
                current_pos_state = hover;
                last_time = ros::Time::now();
            }
        }
        case hover:
        {
            local_pos_pub.publish(pose_pub);
            if(ros::Time::now() - last_time > ros::Duration(10.0))
            {
                current_pos_state = land;
                last_time = ros::Time::now();
            }
        }
        break;
        case land:
        {
            if(current_state.mode == "OFFBOARD"  && 
                   current_state.mode != "AUTO.LAND" && 
                   (ros::Time::now() - landing_last_request > ros::Duration(3.0)))
                {
                    if(land_client.call(landing_cmd) && landing_cmd.response.success)
                    {
                        ROS_INFO("AUTO LANDING");
                    }
                    landing_last_request = ros::Time::now();
                }
        }
        break;
    }
}