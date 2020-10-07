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
#include <state_machine/positionXY.h>
#include <std_msgs/String.h>
#include "math.h"

//for take off
ros::Publisher communication_test_pub;
ros::Publisher take_off_command_pub;
ros::Publisher position_delta_pub;
ros::Publisher land_command_pub;

state_machine::requestCommand_L2F takeOff_command;
state_machine::requestCommand_L2F communication_test_request;
state_machine::requestCommand_L2F land_command;

ros::Time in_place_A_time;
ros::Time in_place_B_time;
ros::Time in_place_C_time;
ros::Time in_place_end_time;

state_machine::positionXY positionDelta;

//state_machine state
//every state need target position

static const int position_A = 1;
static const int position_B = 2;
static const int position_B_hover = 3;
static const int position_C = 4;
static const int position_C_hover = 5;
static const int position_end = 6;

int current_start_point = position_A;

bool position_lead = false;

void position_lead_function();

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

std_msgs::String current_position_state_uav1;
void uav1_cps_rb(const std_msgs::String::ConstPtr& msg)
{
    current_position_state_uav1 = *msg;
}

std_msgs::String current_position_state_uav2;
void uav2_cps_rb(const std_msgs::String::ConstPtr& msg)
{
    current_position_state_uav2 = *msg;
}

std_msgs::String current_position_state_uav3;
void uav3_cps_rb(const std_msgs::String::ConstPtr& msg)
{
    current_position_state_uav3 = *msg;
}

std_msgs::String current_position_state_uav4;
void uav4_cps_rb(const std_msgs::String::ConstPtr& msg)
{
    current_position_state_uav4 = *msg;
}


/*****************************main function*****************************/
int main(int argc, char **argv)
{
    ros::init(argc, argv, "commander_node");
    ros::NodeHandle nh;

    takeOff_command.value = 1;
    land_command.value = 1;
    communication_test_request.value = 1;
 
    positionDelta.x = 0.0;
    positionDelta.y = 0.0;

    ros::Subscriber communication_status_sub1 = nh.subscribe<state_machine::attributeStatus_F2L>("uav1_communication_test_reult",10,uav1_ctr_rb);
    ros::Subscriber communication_status_sub2 = nh.subscribe<state_machine::attributeStatus_F2L>("uav2_communication_test_reult",10,uav2_ctr_rb);
    ros::Subscriber communication_status_sub3 = nh.subscribe<state_machine::attributeStatus_F2L>("uav3_communication_test_reult",10,uav3_ctr_rb);
    ros::Subscriber communication_status_sub4 = nh.subscribe<state_machine::attributeStatus_F2L>("uav4_communication_test_reult",10,uav4_ctr_rb);
    
    ros::Subscriber takeOff_status_sub1 = nh.subscribe<state_machine::attributeStatus_F2L>("uav1_take_off_status",10,uav1_tos_rb);
    ros::Subscriber takeOff_status_sub2 = nh.subscribe<state_machine::attributeStatus_F2L>("uav2_take_off_status",10,uav2_tos_rb);
    ros::Subscriber takeOff_status_sub3 = nh.subscribe<state_machine::attributeStatus_F2L>("uav3_take_off_status",10,uav3_tos_rb);
    ros::Subscriber takeOff_status_sub4 = nh.subscribe<state_machine::attributeStatus_F2L>("uav4_take_off_status",10,uav4_tos_rb);

    ros::Subscriber current_position_state_sub1 = nh.subscribe<std_msgs::String>("uav1_current_position_state",10,uav1_cps_rb);
    ros::Subscriber current_position_state_sub2 = nh.subscribe<std_msgs::String>("uav2_current_position_state",10,uav2_cps_rb);
    ros::Subscriber current_position_state_sub3 = nh.subscribe<std_msgs::String>("uav3_current_position_state",10,uav3_cps_rb);
    ros::Subscriber current_position_state_sub4 = nh.subscribe<std_msgs::String>("uav4_current_position_state",10,uav4_cps_rb);

    take_off_command_pub = nh.advertise<state_machine::requestCommand_L2F>("take_off_command",10);
    land_command_pub = nh.advertise<state_machine::requestCommand_L2F>("land_command",10);
    communication_test_pub = nh.advertise<state_machine::requestCommand_L2F>("communication_test",10);
    position_delta_pub = nh.advertise<state_machine::positionXY>("position_delta",10);

    ros::Rate rate(20.0);

    while(ros::ok())
    {
        static int communication_display = true;
        static int takeoff_display = true;
        static int in_A_place_display = true;

        while(communication_test_request.value == 1)
        {   
            if(communication_display == true)
            {
                ROS_INFO("request for communication test.");
                communication_display = false;
            }
            communication_test_pub.publish(communication_test_request);
            if(communication_status_uav1.value && communication_status_uav2.value && communication_status_uav3.value && communication_status_uav4.value)
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
            if(takeOff_status_uav1.value && takeOff_status_uav2.value && takeOff_status_uav3.value && takeOff_status_uav4.value)
            {
                ROS_INFO("all of the uavs have been taken off.");
                takeOff_command.value = 0;
            }
            ros::spinOnce();
            rate.sleep();
        }
        
        if( current_position_state_uav1.data == "position_A_hover" && 
            current_position_state_uav2.data == "position_A_hover" && 
            current_position_state_uav3.data == "position_A_hover" && 
            current_position_state_uav4.data == "position_A_hover" && 
            in_A_place_display)
        {
            ROS_INFO("all the uavs have been arrived in position A.");
            position_lead = true;
            current_start_point = position_A;
            in_place_A_time = ros::Time::now();
            in_A_place_display = false;
        }

        if(position_lead == true)
        {
            position_lead_function();
        }

        position_delta_pub.publish(positionDelta);
        
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

void position_lead_function()
{
    static int in_B_place_display = true;
    static int in_C_place_display = true;
    static int in_END_place_display = true;

    switch(current_start_point)
    {
        case position_A:
        {
            if(ros::Time::now() - in_place_A_time > ros::Duration(5.0))
            {
                if(positionDelta.y < 5.0)
                {
                    positionDelta.x += 0.0;
                    positionDelta.y += 0.05;
                } 
                if (current_position_state_uav1.data == "position_B_hover" && 
                    current_position_state_uav2.data == "position_B_hover" && 
                    current_position_state_uav3.data == "position_B_hover" &&
                    current_position_state_uav4.data == "position_B_hover" &&
                    in_B_place_display) 
                {
                    ROS_INFO("all the uavs have been arrived in position B.");
                    current_start_point = position_B;
                    in_B_place_display = false;
                    in_place_B_time = ros::Time::now();
                }
            }
        }
        break;
        case position_B:
        {
            if(ros::Time::now() - in_place_B_time > ros::Duration(5.0)) 
            {
                if(positionDelta.x < 5.0)
                {
                    // ROS_INFO("positionDelta.x %f:", positionDelta.x);
                    positionDelta.x += 0.05;
                    positionDelta.y += 0.0;
                }
                if (current_position_state_uav1.data == "position_C_hover" &&
                    current_position_state_uav2.data == "position_C_hover" &&
                    current_position_state_uav3.data == "position_C_hover" &&
                    current_position_state_uav4.data == "position_C_hover" && 
                    in_C_place_display) 
                {
                    ROS_INFO("all the uavs have been arrived in position C.");
                    current_start_point = position_C;
                    in_C_place_display = false;
                    in_place_C_time = ros::Time::now();
                }
            }
        }
        break;
        case position_C:
        {
            if(ros::Time::now() - in_place_C_time > ros::Duration(5.0)) 
            {
                if(positionDelta.x > 0 && positionDelta.y > 0)
                {
                    positionDelta.x -= 0.05;
                    positionDelta.y -= 0.05;
                }
                if( current_position_state_uav1.data == "position_end_hover" && 
                    current_position_state_uav2.data == "position_end_hover" && 
                    current_position_state_uav3.data == "position_end_hover" && 
                    current_position_state_uav4.data == "position_end_hover" && 
                    in_END_place_display) 
                {
                    ROS_INFO("all the uavs have been arrived in position END.");
                    current_start_point = position_end;
                    in_END_place_display = false;
                    in_place_end_time = ros::Time::now();
                }
            }
        }
        break;
        case position_end:
        {
            if(ros::Time::now() - in_place_end_time > ros::Duration(5.0)) 
            {
                ROS_INFO("send land command.");
                land_command_pub.publish(land_command);
                position_lead = false;
            }
        }
        break;
    }
}