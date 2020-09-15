/*************************************************************************
@file           state_machine_demo.cpp
@date           2018/08/20 16:25
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
ros::Publisher communication_test_pub;
ros::Publisher take_off_command_pub;
state_machine::requestCommand_L2F takeOff_command;
state_machine::requestCommand_L2F communication_test_request;

/***************************callback function definition***************/
state_machine::attributeStatus_F2L communication_status_uav1;
void uav1_ctr_rb(const state_machine::attributeStatus_F2L::ConstPtr& msg)
{
    communication_status_uav1 = *msg;
}

state_machine::attributeStatus_F2L communication_status_uav2;
void uav2_ctr_rb(const state_machine::attributeStatus_F2L::ConstPtr& msg)
{
    communication_status_uav2 = *msg;
}

state_machine::attributeStatus_F2L communication_status_uav3;
void uav3_ctr_rb(const state_machine::attributeStatus_F2L::ConstPtr& msg)
{
    communication_status_uav3 = *msg;
}

state_machine::attributeStatus_F2L communication_status_uav4;
void uav4_ctr_rb(const state_machine::attributeStatus_F2L::ConstPtr& msg)
{
    communication_status_uav4 = *msg;
}

state_machine::attributeStatus_F2L takeOff_status_uav1;
void uav1_tos_rb(const state_machine::attributeStatus_F2L::ConstPtr& msg)
{
    takeOff_status_uav1 = *msg;
}

state_machine::attributeStatus_F2L takeOff_status_uav2;
void uav2_tos_rb(const state_machine::attributeStatus_F2L::ConstPtr& msg)
{
    takeOff_status_uav2 = *msg;
}

state_machine::attributeStatus_F2L takeOff_status_uav3;
void uav3_tos_rb(const state_machine::attributeStatus_F2L::ConstPtr& msg)
{
    takeOff_status_uav3 = *msg;
}

state_machine::attributeStatus_F2L takeOff_status_uav4;
void uav4_tos_rb(const state_machine::attributeStatus_F2L::ConstPtr& msg)
{
    takeOff_status_uav4 = *msg;
}


/*****************************main function*****************************/
int main(int argc, char **argv)
{
    ros::init(argc, argv, "commander_node");
    ros::NodeHandle nh;

    takeOff_command.value = 1;
    communication_test_request.value = 1;


    ros::Subscriber communication_status_sub1 = nh.subscribe<state_machine::attributeStatus_F2L>("uav1_communication_test_reult",10,uav1_ctr_rb);
    ros::Subscriber communication_status_sub2 = nh.subscribe<state_machine::attributeStatus_F2L>("uav2_communication_test_reult",10,uav2_ctr_rb);
    ros::Subscriber communication_status_sub3 = nh.subscribe<state_machine::attributeStatus_F2L>("uav3_communication_test_reult",10,uav3_ctr_rb);
    ros::Subscriber communication_status_sub4 = nh.subscribe<state_machine::attributeStatus_F2L>("uav4_communication_test_reult",10,uav4_ctr_rb);
    
    ros::Subscriber takeOff_status_sub1 = nh.subscribe<state_machine::attributeStatus_F2L>("uav1_take_off_status",10,uav1_tos_rb);
    ros::Subscriber takeOff_status_sub2 = nh.subscribe<state_machine::attributeStatus_F2L>("uav2_take_off_status",10,uav2_tos_rb);
    ros::Subscriber takeOff_status_sub3 = nh.subscribe<state_machine::attributeStatus_F2L>("uav3_take_off_status",10,uav3_tos_rb);
    ros::Subscriber takeOff_status_sub4 = nh.subscribe<state_machine::attributeStatus_F2L>("uav4_take_off_status",10,uav4_tos_rb);

    take_off_command_pub = nh.advertise<state_machine::requestCommand_L2F>("take_off_command",10);
    communication_test_pub = nh.advertise<state_machine::requestCommand_L2F>("communication_test",10);

    ros::Rate rate(20.0);

    while(ros::ok())
    {
        static int communication_display = true;
        static int takeoff_display = true;
        while(communication_test_request.value == 1)
        {   
            if(communication_display == true)
            {
                ROS_INFO("request for communication test.");
                communication_display = false;
            }
            communication_test_pub.publish(communication_test_request);
            if(communication_status_uav1.value)// && communication_status_uav3.value && communication_status_uav4.value)
            {
                ROS_INFO("all of the communication test successfully.");
                communication_test_request.value = 0;
            }
            ros::spinOnce();
            rate.sleep();
        }

        while(takeOff_command.value == 1)
        {   
            if(takeoff_display == true)
            {
                ROS_INFO("send take off command.");
                takeoff_display = false;
            }
            take_off_command_pub.publish(takeOff_command);
            if(takeOff_status_uav1.value)// && takeOff_status_uav3.value && takeOff_status_uav4.value)
            {
                ROS_INFO("all of the uavs have been taken off.");
                takeOff_command.value = 0;
            }
            ros::spinOnce();
            rate.sleep();
        }
        
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}