/*************************************************************************
@file           state_machine_v1.1.cpp
@date           2018/09/24 11:01
@author         liuxu
@email          liuxu.ccc@gmail.com
@description    fixed 3 position from GCS;distance measured by sensor;vision position
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

#include <state_machine/Distance.h> 
#include <state_machine/Vision_Position_Raw.h> 


/***************************function declare****************************/

void state_machine_fun(void);


/***************************variable definition*************************/
//position  ENU
geometry_msgs::PoseStamped position_home;
geometry_msgs::PoseStamped position_componnet;
geometry_msgs::PoseStamped position_construction;
geometry_msgs::PoseStamped position_by_calculated;

//topic
geometry_msgs::PoseStamped pose_pub; 
geometry_msgs::TwistStamped vel_pub;

ros::Publisher local_pos_pub;
ros::Publisher local_vel_pub;
ros::Publisher fixed_target_pub;
ros::Publisher vision_position_pub;

//bool velocity_control_enable = true;

//state_machine state,every state need target position

static const int initial = 1;
static const int vision_process = 2;

//mission 
int loop = 0;
int current_pos_state = initial;

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


bool display_enable = false;
bool initial_enable = false;

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

// state_machine::Distance distance;
// void distance_cb(const state_machine::Distance::ConstPtr& msg)
// {
//     distance = *msg;
//     ROS_INFO("distance:%f",distance.distance);
//     if(display_enable == true)
//     {
//         //ROS_INFO("distance:%f",distance.distance);
//         display_enable = false;
//     } 
// }

state_machine::Vision_Position_Raw vision_position_raw;
void vision_position_cb(const state_machine::Vision_Position_Raw::ConstPtr& msg)
{
    vision_position_raw = *msg;

    vision_position_get.loop_value = loop;
    //NED
    vision_position_get.component_position_x = vision_position_raw.x + current_position.pose.position.y;
    vision_position_get.component_position_y = vision_position_raw.y + current_position.pose.position.x;
    vision_position_get.component_position_z = vision_position_raw.z - current_position.pose.position.z;

    vision_position_pub.publish(vision_position_get);

    if(display_enable == true)
    {
        ROS_INFO("loop:%d",vision_position_get.loop_value);
        ROS_INFO("x:%f",vision_position_get.component_position_x);
        ROS_INFO("y:%f",vision_position_get.component_position_y);
        ROS_INFO("z:%f",vision_position_get.component_position_z);
        display_enable = false;
    } 
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
    //ros::Subscriber distance_sub = nh.subscribe<state_machine::Distance>("distance",10,distance_cb);
    ros::Subscriber vision_position_sub = nh.subscribe<state_machine::Vision_Position_Raw>("vision_position",10,vision_position_cb);

    //topic publish
    local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local",10);
    local_vel_pub = nh.advertise<geometry_msgs::TwistStamped>("mavros/setpoint_velocity/cmd_vel",10);
    fixed_target_pub = nh.advertise<state_machine::FIXED_TARGET_RETURN_M2P>("mavros/fixed_target_return_m2p",10);
    ros::Publisher grab_status_pub = nh.advertise<state_machine::GRAB_STATUS_M2P>("mavros/grab_status_m2p",10);
    ros::Publisher task_status_pub = nh.advertise<state_machine::TASK_STATUS_MONITOR_M2P>("mavros/task_status_monitor_m2p",10);
    vision_position_pub = nh.advertise<state_machine::VISION_POSITION_GET_M2P>("mavros/vision_position_get_m2p",10);
    ros::Publisher yaw_sp_pub = nh.advertise<state_machine::YAW_SP_CALCULATED_M2P>("mavros/yaw_sp_calculated_m2p",10);

    //land service
    land_client = nh.serviceClient<state_machine::CommandTOL>("mavros/cmd/land");
    landing_cmd.request.min_pitch = 1.0;
    landing_last_request = ros::Time::now();
    

    ros::Rate rate(10.0);

    // while(ros::ok() && !current_state.connected)
    // {
    //     ros::spinOnce();
    //     rate.sleep();
    // }

    // ROS_INFO("Connect successfully!!");

    // ROS_INFO("send setpoint before takeoff,please wait");

    // for(int i =100; ros::ok() && i > 0; i--)
    // {
    //     local_vel_pub.publish(vel_pub);
    //     ros::spinOnce();
    //     rate.sleep();
    // }

    // ROS_INFO("Initialization finished");

    while(ros::ok())
    {
        state_machine_fun();
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
        case initial:
        {
            display_enable = true;
            // ros::spinOnce();
            initial_enable = true;
            current_pos_state = vision_process;
            last_time = ros::Time::now();
        }
        break;
        case vision_process:
        {
            static int vision_count = 11;
            static int row_count_x_max = 0;
            static int row_count_x_min = 0;
            static int row_count_y_max = 0;
            static int row_count_y_min = 0;
            static float position_x_max = 0;
            static float position_y_max = 0;
            static float position_x_min = 0;
            static float position_y_min = 0;
            static float position_x_total = 0;
            static float position_y_total = 0;
            static float position_x_average = 0;
            static float position_y_average = 0;
            static float position[10][2] = {0};
            
            if (ros::Time::now() - last_time > ros::Duration(1.0) && vision_count > 10)
            {
                vision_count = 0;
            }
            if(ros::Time::now() - last_time > ros::Duration(0.5) && vision_count < 10)
            {
                //ros::spinOnce();
                // position_x_average = (position_x_average * vision_count + vision_position.component_position_x)/(vision_count);
                // position_y_average = (position_x_average * vision_count + vision_position.component_position_y)/(vision_count);
                
                position[vision_count][0] = vision_position_get.component_position_x;
                position[vision_count][1] = vision_position_get.component_position_y;

                if(initial_enable == true)
                {
                    position_x_max = position[0][0];
                    position_y_max = position[0][1];
                    position_x_min = position[0][0];
                    position_y_min = position[0][1];
                    initial_enable = false;
                }

                if(vision_position_get.component_position_x > position_x_max)
                {
                    position_x_max = vision_position_get.component_position_x;
                    row_count_x_max = vision_count;
                }
                if(vision_position_get.component_position_y > position_y_max)
                {
                    position_y_max = vision_position_get.component_position_y;
                    row_count_y_max = vision_count;
                }
                if(vision_position_get.component_position_x < position_x_min)
                {
                    position_x_min = vision_position_get.component_position_x;
                    row_count_x_min = vision_count;
                }
                if(vision_position_get.component_position_y < position_y_min)
                {
                    position_y_min = vision_position_get.component_position_y;
                    row_count_y_min = vision_count;
                }
                 
                vision_count++;
            }
            else if(vision_count == 10)
            {

                for(int i = 0;i < 10;i++)
                {
                    ROS_INFO("position[%d][0]:%f",i,position[i][0]);
                    if(i != row_count_x_max && i != row_count_x_min)
                    {
                        position_x_total += position[i][0]; 
                    }
                           
                }
                for(int i = 0;i < 10;i++)
                {
                    if(i != row_count_x_max && i != row_count_x_min)
                    {
                        position_y_total += position[i][1];
                        ROS_INFO("position[%d][1]:%f",i,position_by_calculated.pose.position.x);
                    }
                }

                ROS_INFO("position_x_max:%f",position_x_max);
                ROS_INFO("row_count_x_max:%d",row_count_x_max);
                ROS_INFO("position_y_max:%f",position_y_max);
                ROS_INFO("row_count_y_max:%d",row_count_y_max);
                ROS_INFO("position_x_min:%f",position_x_min);
                ROS_INFO("row_count_x_min:%d",row_count_x_min);
                ROS_INFO("position_y_min:%f",position_y_min);
                ROS_INFO("row_count_y_min:%d",row_count_y_min);


                position_x_average = position_x_total/8;
                position_y_average = position_y_total/8;

                position_by_calculated.pose.position.x = position_x_average;
                position_by_calculated.pose.position.y = position_y_average;
                position_by_calculated.pose.position.z = 2.5;

                ROS_INFO("position_x_average.x:%f",position_x_average);
                ROS_INFO("position_by_calculated.y:%f",position_by_calculated.pose.position.y);
                ROS_INFO("position_by_calculated.z:%f",position_by_calculated.pose.position.z);

                vision_count = 11;
                row_count_x_max = 0;
                row_count_x_min = 0;
                row_count_y_max = 0;
                row_count_y_min = 0;
                position_x_average = 0;
                position_y_average = 0;
                position_x_max = 0;
                position_y_max = 0;
                position_x_min = 0;
                position_y_min = 0;
                position_x_total = 0;
                position_y_total = 0;
                position[10][2] = {0};
                current_pos_state = initial;
                display_enable = true;
            }
        }
        break;
    }
}