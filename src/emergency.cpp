/*************************************************************************
@file           emergency.cpp
@date           2020/09/16 16:25
@author         liuxu
@email          liuxu.ccc@gmail.com
@description    a simple state machine for the drone race in 2018
                takeoff-->A-->hover-->B-->hover-->C-->hover-->landing
*************************************************************************/

#include <ros/ros.h>
#include <state_machine/attributeStatus_F2L.h>
#include <state_machine/requestCommand_L2F.h>
#include "math.h"

//for take off
ros::Publisher emergency_command_pub;
state_machine::requestCommand_L2F emergency_command;

/***************************callback function definition***************/
state_machine::attributeStatus_F2L emergency_status_uav1;
void uav1_es_rb(const state_machine::attributeStatus_F2L::ConstPtr& msg)
{
    emergency_status_uav1 = *msg;
}

state_machine::attributeStatus_F2L emergency_status_uav2;
void uav2_es_rb(const state_machine::attributeStatus_F2L::ConstPtr& msg)
{
    emergency_status_uav2 = *msg;
}

state_machine::attributeStatus_F2L emergency_status_uav3;
void uav3_es_rb(const state_machine::attributeStatus_F2L::ConstPtr& msg)
{
    emergency_status_uav3 = *msg;
}

state_machine::attributeStatus_F2L emergency_status_uav4;
void uav4_es_rb(const state_machine::attributeStatus_F2L::ConstPtr& msg)
{
    emergency_status_uav4 = *msg;
}


/*****************************main function*****************************/
int main(int argc, char **argv)
{
    ros::init(argc, argv, "emergency_node");
    ros::NodeHandle nh;

    emergency_command.value = 1;
    
    ros::Subscriber takeOff_status_sub1 = nh.subscribe<state_machine::attributeStatus_F2L>("uav1_emergency_status",10,uav1_es_rb);
    ros::Subscriber takeOff_status_sub2 = nh.subscribe<state_machine::attributeStatus_F2L>("uav2_emergency_status",10,uav2_es_rb);
    ros::Subscriber takeOff_status_sub3 = nh.subscribe<state_machine::attributeStatus_F2L>("uav3_emergency_status",10,uav3_es_rb);
    ros::Subscriber takeOff_status_sub4 = nh.subscribe<state_machine::attributeStatus_F2L>("uav4_emergency_status",10,uav4_es_rb);

    emergency_command_pub = nh.advertise<state_machine::requestCommand_L2F>("emergency_command",10);

    ros::Rate rate(20.0);

    while(ros::ok())
    {
        static int emergency_display = true;
        while(emergency_command.value == 1)
        {   
            if(emergency_display == true)
            {
                ROS_INFO("send emergency command.");
                emergency_display = false;
            }
            emergency_command_pub.publish(emergency_command);
            if(emergency_status_uav1.value && emergency_status_uav2.value)// && takeOff_status_uav3.value && takeOff_status_uav4.value)
            {
                ROS_INFO("all of the uavs have been received emergency.");
                emergency_command.value = 0;
            }
            ros::spinOnce();
            rate.sleep();
        }
        
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}