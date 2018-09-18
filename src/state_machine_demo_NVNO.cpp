/*************************************************************************
@file           state_machine_demo.cpp
@date           2018/08/20 16:25
@author         liuxu
@email          liuxu.ccc@gmail.com
@description    a simple state machine for the drone race in 2018
                takeoff-->A-->hover-->B-->hover-->C-->hover-->landing
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
//position  ENU
geometry_msgs::PoseStamped position_A;
geometry_msgs::PoseStamped position_B;
geometry_msgs::PoseStamped position_C;

geometry_msgs::PoseStamped pose_pub;  //ENU
ros::Publisher local_pos_pub;

//state_machine state
//every state need target position

static const int takeoff = 1;
static const int position_A_go = 2;
static const int position_A_hover = 3;
static const int position_B_go = 4;
static const int position_B_hover = 5;
static const int position_C_go = 6;
static const int position_C_hover = 7;
static const int return_home = 8;
static const int land = 9;

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

    //position of A
    position_A.pose.position.x = 0;
    position_A.pose.position.y = 0;
    position_A.pose.position.z = 5;
    //position of B
    position_B.pose.position.x = 0;
    position_B.pose.position.y = 5;
    position_B.pose.position.z = 5;
    //position of C
    position_C.pose.position.x = 5;
    position_C.pose.position.y = 5;
    position_C.pose.position.z = 5;
    
    ros::Subscriber state_sub = nh.subscribe<state_machine::State>("mavros/state",10,state_cb);
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose",10,pose_cb);

    local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local",10);
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
        local_pos_pub.publish(pose_pub);
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
            current_pos_state = position_A_go;
            last_time = ros::Time::now();
        }
        break;
        case position_A_go:
        {
            local_pos_pub.publish(position_A);
            if (abs(current_position.pose.position.x - position_A.pose.position.x) < 1.5 &&
                abs(current_position.pose.position.y - position_A.pose.position.y) < 1.5 &&
                abs(current_position.pose.position.z - position_A.pose.position.z) < 1.5 )
            {
                current_pos_state = position_A_hover;
                last_time = ros::Time::now();
            }
        }
        break;
        case position_A_hover:
        {
            local_pos_pub.publish(position_A);
            if(ros::Time::now() - last_time > ros::Duration(10.0))
            {
                current_pos_state = position_B_go;
                last_time = ros::Time::now();
            }
        }
        break;
        case position_B_go:
        {
            local_pos_pub.publish(position_B);
            if (abs(current_position.pose.position.x - position_B.pose.position.x) < 1.5 &&
                abs(current_position.pose.position.y - position_B.pose.position.y) < 1.5 &&
                abs(current_position.pose.position.z - position_B.pose.position.z) < 1.5 )
            {
                current_pos_state = position_B_hover;
                last_time = ros::Time::now();
            }
        }
        break;
        case position_B_hover:
        {
            local_pos_pub.publish(position_B);
            if(ros::Time::now() - last_time > ros::Duration(10.0))
            {
                current_pos_state = position_C_go;
                last_time = ros::Time::now();
            }
        }
        break;
        case position_C_go:
        {
            local_pos_pub.publish(position_C);
            if (abs(current_position.pose.position.x - position_C.pose.position.x) < 1.5 &&
                abs(current_position.pose.position.y - position_C.pose.position.y) < 1.5 &&
                abs(current_position.pose.position.z - position_C.pose.position.z) < 1.5 )
            {
                current_pos_state = position_C_hover;
                last_time = ros::Time::now();
            }
        }
        break;
        case position_C_hover:
        {
            local_pos_pub.publish(position_C);
            if(ros::Time::now() - last_time > ros::Duration(10.0))
            {
                current_pos_state = return_home;
                last_time = ros::Time::now();
            }
        }
        break;
        case  return_home:
        {
            local_pos_pub.publish(position_A);
            if (abs(current_position.pose.position.x - position_A.pose.position.x) < 1.5 &&
                abs(current_position.pose.position.y - position_A.pose.position.y) < 1.5 &&
                abs(current_position.pose.position.z - position_A.pose.position.z) < 1.5 )
            {
                current_pos_state = land;
                last_time = ros::Time::now();
            }
        }
        break;
        case land:
        {
            if(current_state.mode != "AUTO.LAND" && 
                   (ros::Time::now() - landing_last_request > ros::Duration(10.0)))
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