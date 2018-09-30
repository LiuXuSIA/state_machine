/*************************************************************************
@file           state_machine_v1.0.cpp
@date           2018/09/24 11:01
@author         liuxu
@email          liuxu.ccc@gmail.com
@description    fixed 3 position from GCS
*************************************************************************/


/****************************header files********************************/
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <state_machine/CommandTOL.h>
#include <state_machine/State.h>
#include "math.h"

#include <state_machine/FIXED_TARGET_POSITION_P2M.h>
#include <state_machine/FIXED_TARGET_RETURN_M2P.h>
#include <state_machine/GRAB_STATUS_M2P.h>
#include <state_machine/TASK_STATUS_CHANGE_P2M.h>
#include <state_machine/TASK_STATUS_MONITOR_M2P.h>
#include <state_machine/VISION_POSITION_GET_M2P.h>
#include <state_machine/YAW_SP_CALCULATED_M2P.h>


/***************************function declare****************************/

void state_machine_fun(void);


/***************************variable definition*************************/
//position  ENU
geometry_msgs::PoseStamped position_home;
geometry_msgs::PoseStamped position_componnet;
geometry_msgs::PoseStamped position_construction;

//topic
geometry_msgs::PoseStamped pose_pub; 
geometry_msgs::TwistStamped vel_pub;

ros::Publisher local_pos_pub;
ros::Publisher local_vel_pub;
ros::Publisher fixed_target_pub;

//bool velocity_control_enable = true;

//state_machine state,every state need target position
static const int takeoff = 1;
static const int position_H_go = 2;
static const int position_H_hover = 3;
static const int position_Com_go = 4;
static const int position_Com_hover = 5;
static const int position_Con_go = 6;
static const int position_Con_hover = 7;
static const int return_home = 8;
static const int land = 9;

//mission 
int loop = 0;
int current_pos_state = takeoff;

//time
ros::Time last_time;

//land
state_machine::CommandTOL landing_cmd;
ros::ServiceClient land_client;
ros::Time landing_last_request;

//message to pix
state_machine::FIXED_TARGET_RETURN_M2P fix_target_return;
state_machine::GRAB_STATUS_M2P grab_status;
state_machine::TASK_STATUS_MONITOR_M2P task_status_monitor;
state_machine::VISION_POSITION_GET_M2P vision_position_get;
state_machine::YAW_SP_CALCULATED_M2P yaw_sp_calculated;


/*************************constant defunition***************************/

#define TAKEOFF_HEIGHT      5
#define TAKEOFF_VELOCITY    2
#define OBSERVE_HEIGET      5


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

geometry_msgs::TwistStamped current_velocity;
void velo_cb(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    current_velocity = *msg;
}

state_machine::FIXED_TARGET_POSITION_P2M fix_target_position;
void fixed_target_position_p2m_cb(const state_machine::FIXED_TARGET_POSITION_P2M::ConstPtr& msg)
{
    fix_target_position = *msg;

    fix_target_return.home_x = fix_target_position.home_x;
    fix_target_return.home_y = fix_target_position.home_y;
    fix_target_return.home_z = TAKEOFF_HEIGHT;

    fix_target_return.component_x = fix_target_position.component_x;
    fix_target_return.component_y = fix_target_position.component_y;
    fix_target_return.component_z = OBSERVE_HEIGET;

    fix_target_return.construction_x = fix_target_position.construction_x;
    fix_target_return.construction_y = fix_target_position.construction_y;
    fix_target_return.construction_z = OBSERVE_HEIGET;

    ROS_INFO("fix_target_return.home_x:%f",fix_target_return.home_x);
    ROS_INFO("fix_target_return.home_y:%f",fix_target_return.home_y);
    ROS_INFO("fix_target_return.home_z:%f",fix_target_return.home_z);

    ROS_INFO("fix_target_return.component_x:%f",fix_target_return.component_x);
    ROS_INFO("fix_target_return.component_y:%f",fix_target_return.component_y);
    ROS_INFO("fix_target_return.component_z:%f",fix_target_return.component_z);

    ROS_INFO("fix_target_return.construction_x:%f",fix_target_return.construction_x);
    ROS_INFO("fix_target_return.construction_y:%f",fix_target_return.construction_y);
    ROS_INFO("fix_target_return.construction_z:%f",fix_target_return.construction_z);

    fixed_target_pub.publish(fix_target_return);

    //Coordinate transformation
    //position of home
    position_home.pose.position.x = fix_target_position.home_y;
    position_home.pose.position.y = fix_target_position.home_x;
    position_home.pose.position.z = TAKEOFF_HEIGHT;
    //position of componnet
    position_componnet.pose.position.x = fix_target_position.component_y;
    position_componnet.pose.position.y = fix_target_position.component_x;
    position_componnet.pose.position.z = OBSERVE_HEIGET;
    //position of construction
    position_construction.pose.position.x = fix_target_position.construction_y;
    position_construction.pose.position.y = fix_target_position.construction_x;
    position_construction.pose.position.z = OBSERVE_HEIGET;
    
    //adjust angular,face north
    float yaw_sp=M_PI_2;
    //home
    position_home.pose.orientation.x = 0;
    position_home.pose.orientation.y = 0;
    position_home.pose.orientation.z = sin(yaw_sp/2);
    position_home.pose.orientation.w = cos(yaw_sp/2);
    //componnet
    position_componnet.pose.orientation.x = 0;
    position_componnet.pose.orientation.y = 0;
    position_componnet.pose.orientation.z = sin(yaw_sp/2);
    position_componnet.pose.orientation.w = cos(yaw_sp/2);
    //construction
    position_construction.pose.orientation.x = 0;
    position_construction.pose.orientation.y = 0;
    position_construction.pose.orientation.z = sin(yaw_sp/2);
    position_construction.pose.orientation.w = cos(yaw_sp/2);
}

state_machine::TASK_STATUS_CHANGE_P2M task_status_change;
void task_status_change_p2m_cb(const state_machine::TASK_STATUS_CHANGE_P2M::ConstPtr& msg)
{
    task_status_change = *msg;
    ROS_INFO("task_status_change.task_status:%d",task_status_change.task_status);
    ROS_INFO("task_status_change.loop_value:%d",task_status_change.loop_value);
}


/*****************************main function*****************************/
int main(int argc, char **argv)
{
    ros::init(argc, argv, "offboard_node");
    ros::NodeHandle nh;

    //takeoff velocity
    vel_pub.twist.linear.x = 0.0f;
    vel_pub.twist.linear.y = 0.0f;
    vel_pub.twist.linear.z = 2.0f;
    vel_pub.twist.angular.x = 0.0f;
    vel_pub.twist.angular.y = 0.0f;
    vel_pub.twist.angular.z = 0.0f;
     
    //topic  subscribe
    ros::Subscriber state_sub = nh.subscribe<state_machine::State>("mavros/state",10,state_cb);
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose",10,pose_cb);
    ros::Subscriber vel_sub = nh.subscribe<geometry_msgs::TwistStamped>("mavros/local_position/velocity",10,velo_cb);
    ros::Subscriber fixed_target_sub = nh.subscribe<state_machine::FIXED_TARGET_POSITION_P2M>("mavros/fixed_target_position_p2m",10,fixed_target_position_p2m_cb);
    ros::Subscriber task_status_sub = nh.subscribe<state_machine::TASK_STATUS_CHANGE_P2M>("mavros/task_status_change_p2m",10,task_status_change_p2m_cb);

    //topic publish
    local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local",10);
    local_vel_pub = nh.advertise<geometry_msgs::TwistStamped>("mavros/setpoint_velocity/cmd_vel",10);
    fixed_target_pub = nh.advertise<state_machine::FIXED_TARGET_RETURN_M2P>("mavros/fixed_target_return_m2p",10);
    ros::Publisher grab_status_pub = nh.advertise<state_machine::GRAB_STATUS_M2P>("mavros/grab_status_m2p",10);
    ros::Publisher task_status_pub = nh.advertise<state_machine::TASK_STATUS_MONITOR_M2P>("mavros/task_status_monitor_m2p",10);
    ros::Publisher vision_position_pub = nh.advertise<state_machine::VISION_POSITION_GET_M2P>("mavros/vision_position_get_m2p",10);
    ros::Publisher yaw_sp_pub = nh.advertise<state_machine::YAW_SP_CALCULATED_M2P>("mavros/yaw_sp_calculated_m2p",10);

    //land service
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

        task_status_monitor.task_status = current_pos_state;
        task_status_monitor.loop_value = loop;
        task_status_monitor.target_x = pose_pub.pose.position.y;
        task_status_monitor.target_y = pose_pub.pose.position.x;
        task_status_monitor.target_z = pose_pub.pose.position.z;
        task_status_pub.publish(task_status_monitor);

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
            pose_pub = position_home;
            local_vel_pub.publish(vel_pub);
            if(current_position.pose.position.z > 4)
            {
                current_pos_state = position_H_go;
                last_time = ros::Time::now();
            }
        }
        break;
        case position_H_go:
        {
            local_pos_pub.publish(position_home);
            pose_pub = position_home;
            if (abs(current_position.pose.position.x - position_home.pose.position.x) < 1 &&
                abs(current_position.pose.position.y - position_home.pose.position.y) < 1 &&
                abs(current_position.pose.position.z - position_home.pose.position.z) < 1 )
            {
                current_pos_state = position_H_hover;
                last_time = ros::Time::now();
            }
        }
        break;
        case position_H_hover:
        {
            pose_pub = position_home;
            local_pos_pub.publish(position_home);
            if(ros::Time::now() - last_time > ros::Duration(10.0))
            {
                current_pos_state = position_Com_go;
                last_time = ros::Time::now();
            }
        }
        break;
        case position_Com_go:
        {
            pose_pub = position_componnet;
            local_pos_pub.publish(position_componnet);
            if (abs(current_position.pose.position.x - position_componnet.pose.position.x) < 1 &&
                abs(current_position.pose.position.y - position_componnet.pose.position.y) < 1 &&
                abs(current_position.pose.position.z - position_componnet.pose.position.z) < 1 )
            {
                current_pos_state = position_Com_hover;
                last_time = ros::Time::now();
            }
        }
        break;
        case position_Com_hover:
        {
            pose_pub = position_componnet;
            local_pos_pub.publish(position_componnet);
            if(ros::Time::now() - last_time > ros::Duration(10.0))
            {
                current_pos_state = position_Con_go;
                last_time = ros::Time::now();
            }
        }
        break;
        case position_Con_go:
        {
            pose_pub = position_construction;
            local_pos_pub.publish(position_construction);
            if (abs(current_position.pose.position.x - position_construction.pose.position.x) < 1 &&
                abs(current_position.pose.position.y - position_construction.pose.position.y) < 1 &&
                abs(current_position.pose.position.z - position_construction.pose.position.z) < 1 )
            {
                current_pos_state = position_Con_hover;
                last_time = ros::Time::now();
            }
        }
        break;
        case position_Con_hover:
        {
            pose_pub = position_construction;
            local_pos_pub.publish(position_construction);
            if(ros::Time::now() - last_time > ros::Duration(10.0))
            {
                current_pos_state = return_home;
                last_time = ros::Time::now();
            }
        }
        break;
        case  return_home:
        {
            pose_pub = position_home;
            local_pos_pub.publish(position_home);
            if (abs(current_position.pose.position.x - position_home.pose.position.x) < 1 &&
                abs(current_position.pose.position.y - position_home.pose.position.y) < 1 &&
                abs(current_position.pose.position.z - position_home.pose.position.z) < 1 )
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