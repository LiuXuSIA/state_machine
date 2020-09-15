/*************************************************************************
@file           state_machine_demo.cpp
@date           2018/09/15
@author         liuxu
@email          liuxu.ccc@gmail.com
@description    just for some test
*************************************************************************/

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <state_machine/State.h>
#include <state_machine/FIXED_TARGET_POSITION_P2M.h>
#include <state_machine/FIXED_TARGET_RETURN_M2P.h>
#include <state_machine/GRAB_STATUS_M2P.h>
#include <state_machine/TASK_STATUS_CHANGE_P2M.h>
#include <state_machine/TASK_STATUS_MONITOR_M2P.h>
#include <state_machine/VISION_POSITION_GET_M2P.h>
#include <state_machine/YAW_SP_CALCULATED_M2P.h>
#include <state_machine/CommandTOL.h>
#include <state_machine/CommandBool.h>
#include <state_machine/SetMode.h>
#include "math.h"

#include <state_machine/Distance.h> 

/***************************function declare****************************/
void state_machine_fun(void);

/**************************for different debug**************************/
#define communication_debug
//#define orientation_debug

/***************************variable definition*************************/
static const int initial = 1;
static const int hover_to_recognize = 2;
int state = initial;

//time
ros::Time last_time;

geometry_msgs::PoseStamped pose_pub;  //ENU
geometry_msgs::TwistStamped vel_pub;
bool receive_flag = false;

state_machine::FIXED_TARGET_RETURN_M2P fix_target_return;
state_machine::GRAB_STATUS_M2P grab_status;
state_machine::TASK_STATUS_MONITOR_M2P task_status_monitor;
state_machine::VISION_POSITION_GET_M2P vision_position_get;
state_machine::YAW_SP_CALCULATED_M2P yaw_sp_calculated;

bool display_enable = false;
bool initial_enable = false;

ros::ServiceClient arming_client;
ros::ServiceClient set_mode_client_offboard;
ros::Time last_request;

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
    /*
    ROS_INFO("current_position.x:%f",current_position.pose.position.x);
    ROS_INFO("current_position.y:%f",current_position.pose.position.y);
    ROS_INFO("current_position.z:%f",current_position.pose.position.z);
    ROS_INFO("current_position.x:%f",current_position.pose.orientation.x);
    ROS_INFO("current_position.y:%f",current_position.pose.orientation.y);
    ROS_INFO("current_position.z:%f",current_position.pose.orientation.z);
    ROS_INFO("current_position.w:%f",current_position.pose.orientation.w);
    */
}

state_machine::FIXED_TARGET_POSITION_P2M fix_target_position;
void fixed_target_position_p2m_cb(const state_machine::FIXED_TARGET_POSITION_P2M::ConstPtr& msg)
{
    fix_target_position = *msg;
    fix_target_return.home_x = fix_target_position.home_x;
    fix_target_return.home_y = fix_target_position.home_y;
    fix_target_return.home_z = fix_target_position.home_z;
    fix_target_return.component_x = fix_target_position.component_x;
    fix_target_return.component_y = fix_target_position.component_y;
    fix_target_return.component_z = fix_target_position.component_z;
    fix_target_return.construction_x = fix_target_position.construction_x;
    fix_target_return.construction_y = fix_target_position.construction_y;
    fix_target_return.construction_z = fix_target_position.construction_z;
    ROS_INFO("fix_target_return.home_x:%f",fix_target_return.home_x);
    ROS_INFO("fix_target_return.home_y:%f",fix_target_return.home_y);
    ROS_INFO("fix_target_return.home_z:%f",fix_target_return.home_z);
    ROS_INFO("fix_target_return.component_x:%f",fix_target_return.component_x);
    ROS_INFO("fix_target_return.component_y:%f",fix_target_return.component_y);
    ROS_INFO("fix_target_return.component_z:%f",fix_target_return.component_z);
    ROS_INFO("fix_target_return.construction_x:%f",fix_target_return.construction_x);
    ROS_INFO("fix_target_return.construction_y:%f",fix_target_return.construction_y);
    ROS_INFO("fix_target_return.construction_z:%f",fix_target_return.construction_z);

    task_status_monitor.task_status = 17;

    receive_flag = true;
}

state_machine::TASK_STATUS_CHANGE_P2M task_status_change;
void task_status_change_p2m_cb(const state_machine::TASK_STATUS_CHANGE_P2M::ConstPtr& msg)
{
    task_status_change = *msg;
    ROS_INFO("task_status_change.task_status:%d",task_status_change.task_status);
    ROS_INFO("task_status_change.loop_value:%d",task_status_change.loop_value);
}

state_machine::Distance distance;
void distance_cb(const state_machine::Distance::ConstPtr& msg)
{
    distance = *msg;
    ROS_INFO("distance:%f",distance.distance);
    if(display_enable == true)
    {
        //ROS_INFO("distance:%f",distance.distance);
        display_enable = false;
    } 
}


/*****************************main function*****************************/
int main(int argc, char **argv)
{
    ros::init(argc, argv, "offboard_node");
    ros::NodeHandle nh;
    
    //adjust angular,face north
    //float yaw_sp=M_PI_2;
    //takeoff velocity
    vel_pub.twist.linear.x = 0.0f;
    vel_pub.twist.linear.y = 0.0f;
    vel_pub.twist.linear.z = 2;
    vel_pub.twist.angular.x = 0.0f;
    vel_pub.twist.angular.y = 0.0f;
    vel_pub.twist.angular.z = 0.0f;

    pose_pub.pose.position.x = 1;
    pose_pub.pose.position.y = 1;
    pose_pub.pose.position.z = 1;
    pose_pub.pose.orientation.x = 1;
    pose_pub.pose.orientation.y = 1;
    pose_pub.pose.orientation.z = 1;
    pose_pub.pose.orientation.w = 1;

    grab_status.grab_status = 2;

    task_status_monitor.task_status = 10;
    task_status_monitor.loop_value = 3;
    task_status_monitor.target_x = 3;
    task_status_monitor.target_y = 3;
    task_status_monitor.target_z = 3;

    vision_position_get.loop_value = 4;
    vision_position_get.component_position_x = 4;
    vision_position_get.component_position_y = 4;
    vision_position_get.component_position_z = 4;

    yaw_sp_calculated.yaw_sp = 5;


    //subscribe
    ros::Subscriber state_sub = nh.subscribe<state_machine::State>("mavros/state",10,state_cb);
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose",10,pose_cb);
    ros::Subscriber fixed_target_sub = nh.subscribe<state_machine::FIXED_TARGET_POSITION_P2M>("mavros/fixed_target_position_p2m",10,fixed_target_position_p2m_cb);
    ros::Subscriber task_status_sub = nh.subscribe<state_machine::TASK_STATUS_CHANGE_P2M>("mavros/task_status_change_p2m",10,task_status_change_p2m_cb);
    ros::Subscriber distance_sub = nh.subscribe<state_machine::Distance>("distance",10,distance_cb);

    //publish
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local",10);
    ros::Publisher local_vel_pub = nh.advertise<geometry_msgs::TwistStamped>("mavros/setpoint_velocity/cmd_vel",10);
    ros::Publisher fixed_target_pub = nh.advertise<state_machine::FIXED_TARGET_RETURN_M2P>("mavros/fixed_target_return_m2p",10);
    ros::Publisher grab_status_pub = nh.advertise<state_machine::GRAB_STATUS_M2P>("mavros/grab_status_m2p",10);
    ros::Publisher task_status_pub = nh.advertise<state_machine::TASK_STATUS_MONITOR_M2P>("mavros/task_status_monitor_m2p",10);
    ros::Publisher vision_position_pub = nh.advertise<state_machine::VISION_POSITION_GET_M2P>("mavros/vision_position_get_m2p",10);
    ros::Publisher yaw_sp_pub = nh.advertise<state_machine::YAW_SP_CALCULATED_M2P>("mavros/yaw_sp_calculated_m2p",10);

    set_mode_client_offboard = nh.serviceClient<state_machine::SetMode>("mavros/set_mode");
    state_machine::SetMode offb_set_mode;
	offb_set_mode.request.custom_mode = "OFFBOARD";
    last_request = ros::Time::now();

    arming_client = nh.serviceClient<state_machine::CommandBool>("mavros/cmd/arming");
	state_machine::CommandBool arm_cmd;
	arm_cmd.request.value = true;
    last_request = ros::Time::now();

    ros::Rate rate(20.0);

    while(ros::ok() && !current_state.connected)
    {
    	ros::spinOnce();
    	rate.sleep();
    }

    ROS_INFO("Connect successfully!!");

    //#ifdef orientation_debug
    ROS_INFO("send setpoint before takeoff,please wait");

    for(int i =100; ros::ok() && i > 0; i--)
	{
		local_pos_pub.publish(pose_pub);
		ros::spinOnce();
		rate.sleep();
	}

    ROS_INFO("Initialization finished");

	while(ros::ok())// && current_state.connected)
	{     
        static bool display_flag = true;

        if(current_state.mode == "OFFBOARD" && current_state.armed)
        {
            if(display_flag == true)
            {
                ROS_INFO("switched to offoard!!");
                display_flag = false;
            }
        }

        local_vel_pub.publish(vel_pub);

        /*
        if(current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0)))
        {
            if( set_mode_client_offboard.call(offb_set_mode) && offb_set_mode.response.success) //old version was success, new version is mode_sent
            {
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        }
        else 
        {
            if(!current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0)))
            {
                if( arming_client.call(arm_cmd) && arm_cmd.response.success)
                {
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }
        */
        if(!current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0)))
        {
            ROS_INFO("arming...");
            if(arming_client.call(arm_cmd) && arm_cmd.response.success) //old version was success, new version is mode_sent
            {
                ROS_INFO("Vehicle armed");
            }
            last_request = ros::Time::now();
        }
        if(current_state.armed && current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0)))
        {
            ROS_INFO("offboarding ...");
            if(set_mode_client_offboard.call(offb_set_mode) && offb_set_mode.response.success)
            {
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now(); 
        }
        ros::spinOnce();
        rate.sleep();
    }
}

void state_machine_fun()
{

    switch(state)
    {
         case initial:
        {
            display_enable = true;
            // ros::spinOnce();
            initial_enable = true;
            state = hover_to_recognize;
            last_time = ros::Time::now();
        }
        break;
        case hover_to_recognize:
        {   
            static float position_x_aver = 0;
            //static float position_y_aver = 0;
            //static float position_z_aver = 0;
            static int vision_count1 = 11;

            if (ros::Time::now() - last_time > ros::Duration(1.0) && vision_count1 > 10)
            {
                vision_count1 = 0;
            }
            if(ros::Time::now() - last_time > ros::Duration(0.5) && vision_count1 < 10)
            {
                ros::spinOnce();
                position_x_aver = (position_x_aver * vision_count1 + distance.distance)/(vision_count1 + 1);
                //position_y_aver = (position_y_aver * vision_count1 + vision_position_get.component_position_y)/(vision_count1 + 1);
               //position_z_aver = (position_z_aver * vision_count1 + vision_position_get.component_position_z)/(vision_count1 + 1);
                // current_pos_state = component_locate;   //need change Z place to component_locate
                vision_count1++;
                //display_enable = true;
            }
            else if(vision_count1 == 10)
            {
                ROS_INFO("position_x_aver:%f",position_x_aver);
                //ROS_INFO("position_y_aver:%f",position_y_aver);
                //ROS_INFO("position_z_aver:%f",position_z_aver);

                // position_box.pose.position.x = position_y_aver;
                // position_box.pose.position.y = position_x_aver;
                // position_box.pose.position.z = RECOGNIZE_HEIGHT - position_z_aver;
                vision_count1 = 11;
                position_x_aver = 0;
                // position_y_aver = 0;
                // position_z_aver = 0;
                state = 4; 
                last_time = ros::Time::now();
            }
        }
        break;
    }
    
}
