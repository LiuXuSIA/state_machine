/*************************************************************************
@file           state_machine_v1.2.cpp
@date           2018/08/20 16:25
@author         liuxu
@email          liuxu.ccc@gmail.com
@description    a simple state machine for the drone race in 2018
*************************************************************************/

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <state_machine/CommandBool.h>
#include <state_machine/CommandTOL.h>
#include <state_machine/SetMode.h>
#include <state_machine/State.h>
#include <state_machine/Attitude.h>
#include <state_machine/Setpoint.h>
#include "math.h"
#include "std_msgs/Int32.h"

/***************************CONSTANT definition*************************/
#define FIXED_POS_HEIGHT      3
#define MAX_MISSION_TIME      20

//#include <mavros/frame_tf.h>

/***************************function declare****************************/
void state_machine_fun(void);
double distance(double x1, double x2, double y1, double y2, double z1, double z2);
float wrap_pi(float angle_rad);

/***************************variable definition*************************/
//position
geometry_msgs::PoseStamped position_H;
geometry_msgs::PoseStamped position_Com;
geometry_msgs::PoseStamped position_Con;

geometry_msgs::PoseStamped pose_pub;  //ENU
geometry_msgs::TwistStamped vel_pub;

bool velocity_control_enable = false;

//state_machine state
//every state need target position

static const int takeoff = 1;
static const int hover_after_takeoff = 2;
static const int hover_only = 3;
static const int component_point_go = 4;
static const int component_recognize = 5;
static const int component_locate = 6;
static const int attitude_adjust_before_suck = 7;
static const int componnet_get_close = 8;
static const int spread_suction_cups = 9;
static const int componnet_grab = 10;
static const int construction_point_go = 11;
static const int place_point_recognize = 12;
static const int place_point_locate = 13;
static const int attitude_adjust_before_place = 14;
static const int place_point_get_close = 15;
//static const int hover_brfore_place = 4;
static const int componnet_place = 16;
static const int place_done = 17;
static const int force_return_home = 18;
static const int return_home = 19;
static const int land = 20;

//mission 
int loop = 0;
int current_mission_state = takeoff;

//time
ros::Time mission_timer_start_time;
ros::Time mission_last_time;
bool mission_timer_enable = true;


bool force_home_enable = true;

/***************************callback function definition***************/
state_machine::State current_state;
state_machine::State last_state;
void state_cb(const state_machine::State::ConstPtr& msg)
{
    last_state = current_state;
    current_state = *msg;
}

state_machine::Attitude att;
void att_cb(const state_machine::Attitude::ConstPtr& msg)
{
    att = *msg;
}

state_machine::Setpoint set3point;
void setpoint_cb(const state_machine::Setpoint::ConstPtr& msg)
{
    set3point = *msg;
    switch(set3point.index)
    {
        case 1:
        position_H.pose.position.x = set3point.x;
        position_H.pose.position.y = set3point.y;
        position_H.pose.position.z = set3point.z;
        break;
        case 2:
        position_Com.pose.position.x = set3point.x;
        position_Com.pose.position.y = set3point.y;
        position_Com.pose.position.z = set3point.z;
        break;
        case 3:
        position_Con.pose.position.x = set3point.x;
        position_Con.pose.position.y = set3point.y;
        position_Con.pose.position.z = set3point.z;
        break;
        default:
        ROS_INFO("ERROR,receive no position");
    }
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

/*****************************main function*****************************/
int main(int argc, char **argv)
{
    ros::init(argc, argv, "offboard_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<state_machine::State>("mavros/state",10,state_cb);
    ros::Subscriber att_sub = nh.subscribe<state_machine::Attitude>("mavros/attitude",10,att_cb);
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose",10,pose_cb);
    ros::Subscriber vel_sub = nh.subscribe<geometry_msgs::TwistStamped>("mavros/local_position/velocity",10,velo_cb);
    ros::Subscriber setpoint_sub = nh.subscribe<state_machine::Setpoint>("set3point",10,setpoint_cb);

    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local",10);
    ros::Publisher local_vel_pub = nh.advertise<geometry_msgs::TwistStamped>("mavros/setpoint_velocity/cmd_vel",10);
    ros::ServiceClient arming_client = nh.serviceClient<state_machine::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<state_machine::SetMode>("mavros/set_mode");
    ros::ServiceClient land_client = nh.serviceClient<state_machine::CommandTOL>("mavros/cmd/land");
    state_machine::CommandTOL landing_cmd;
    landing_cmd.request.min_pitch = 1.0;
    ros::Time landing_last_request = ros::Time::now();

    ros::Rate rate(10.0);

    while(ros::ok() && !current_state.connected)
    {
    	ros::spinOnce();
    	rate.sleep();
    }

    ROS_INFO("Connect successfully!!");

    vel_pub.twist.linear.x = 0.0f;
    vel_pub.twist.linear.y = 0.0f;
    vel_pub.twist.linear.z = 1.0f;
    vel_pub.twist.angular.x = 0.0f;
    vel_pub.twist.angular.y = 0.0f;
    vel_pub.twist.angular.z = 0.0f;

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
        /******************************************Mode Change*************************************/
        if(current_state.mode == "MANUAL" && last_state.mode != "MANUAL")
        {
            last_state.mode = "MANUAL";
            ROS_INFO("switch to mode : MANUAL");
        }
        if(current_state.mode == "ALTCTL" && last_state.mode != "ALTCTL")
        {
            last_state.mode = "ALTCTL";
            ROS_INFO("switch to mode: ALTCTL");
        }
        if(current_state.mode == "OFFBOARD" && last_state.mode != "OFFBOARD")
        {
            last_state.mode = "OFFBOARD";
            ROS_INFO("switch to mode: OFFBOARD");
        }
        if(current_state.armed && !last_state.armed)
        {
            last_state.armed = current_state.armed;
            ROS_INFO("UAV armed!");
        }

        if(current_state.mode == "MANUAL" && current_state.armed)
        {
            NULL;
        }

        if(current_state.mode == "OFFBOARD" && current_state.armed)
        {
            state_machine_fun();
            ROS_INFO("current_mission_state:%d",current_mission_state);
            ROS_INFO("loop:%d",loop);
            if(ros::Time::now() - mission_timer_start_time > ros::Duration(MAX_MISSION_TIME) && force_home_enable == true)
            {
                force_home_enable = false;
                current_mission_state = force_return_home;
                mission_last_time = ros::Time::now();
                ROS_INFO("Mission time run out,will return home and landing!!");
            }
            if(current_mission_state == land)
            {
                if(current_state.mode != "AUTO.LAND" && 
                   (ros::Time::now() - landing_last_request > ros::Duration(5)))
                {
                    if(land_client.call(landing_cmd) && landing_cmd.response.success)
                    {
                        ROS_INFO("AUTO LANDING");
                    }
                    landing_last_request = ros::Time::now();
                }
            }
        }
        if(velocity_control_enable)
        {
            local_vel_pub.publish(vel_pub);
        }
        else
        {
            local_pos_pub.publish(pose_pub);
        }
		ros::spinOnce();
		rate.sleep();
	}

	return 0;
}

/***********************************state_machine function***********************************/
void state_machine_fun(void)
{
    switch(current_mission_state)
    {
        case takeoff:
        {
            velocity_control_enable = true;
            vel_pub.twist.linear.x = 0.0f;
            vel_pub.twist.linear.y = 0.0f;
            vel_pub.twist.linear.z = 1.5f;
            vel_pub.twist.angular.x = 0.0f;
            vel_pub.twist.angular.y = 0.0f;
            vel_pub.twist.angular.z = 0.0f;
            if(mission_timer_enable)
            {
                mission_timer_start_time = ros::Time::now();
                mission_timer_enable = false;
            }
            if(current_velocity.twist.linear.z > 0.5 && current_position.pose.position.z > 2)
            {
                current_mission_state = hover_after_takeoff;
                mission_last_time = ros::Time::now();

                velocity_control_enable = false;
                pose_pub.pose.position.x = current_position.pose.position.x;
                pose_pub.pose.position.y = current_position.pose.position.y;
                pose_pub.pose.position.z = position_H.pose.position.z;
            }
        }
        break;
        case hover_after_takeoff:
        {
            pose_pub.pose.position.x = current_position.pose.position.x;
            pose_pub.pose.position.y = current_position.pose.position.y;
            pose_pub.pose.position.z = position_H.pose.position.z;
            if(ros::Time::now() - mission_last_time > ros::Duration(5))
            {
                current_mission_state = component_point_go;
            }
        }
        break;
        case component_point_go:
        {
            pose_pub.pose.position.x = position_Com.pose.position.x;
            pose_pub.pose.position.y = position_Com.pose.position.y;
            pose_pub.pose.position.z = position_Com.pose.position.z;
            if (distance(current_position.pose.position.x,pose_pub.pose.position.x,
                        current_position.pose.position.y,pose_pub.pose.position.y,
                        current_position.pose.position.z,pose_pub.pose.position.z) < 0.2)
            {
                current_mission_state = construction_point_go;
                mission_last_time = ros::Time::now();
            }
        }
        break;
        case construction_point_go:
        {
            pose_pub.pose.position.x = position_Con.pose.position.x;
            pose_pub.pose.position.y = position_Con.pose.position.y;
            pose_pub.pose.position.z = position_Con.pose.position.z;
            if (distance(current_position.pose.position.x,pose_pub.pose.position.x,
                         current_position.pose.position.y,pose_pub.pose.position.y,
                         current_position.pose.position.z,pose_pub.pose.position.z) < 0.2)
            {
                current_mission_state = place_done;
                mission_last_time = ros::Time::now();
            }
        }
        case place_done:
        {
            loop++;
            if(loop < 100)
            {
                current_mission_state = component_point_go;
            }
            else
            {
                current_mission_state = return_home;
            }
        }
        case force_return_home:
        {
            pose_pub.pose.position.x = current_position.pose.position.x;
            pose_pub.pose.position.y = current_position.pose.position.y;
            pose_pub.pose.position.z = current_position.pose.position.z;
            if(ros::Time::now() - mission_last_time >ros::Duration(6))
            {
                current_mission_state = return_home;
            }
        }
        break;
        case  return_home:
        {
            pose_pub.pose.position.x = position_H.pose.position.x;
            pose_pub.pose.position.y = position_H.pose.position.y;
            pose_pub.pose.position.z = position_H.pose.position.z;
            if((abs(current_position.pose.position.x - position_H.pose.position.x) < 0.2) &&
               (abs(current_position.pose.position.y - position_H.pose.position.y) < 0.2) &&
               (abs(current_position.pose.position.z - position_H.pose.position.z) < 0.2))
            {
                current_mission_state = hover_only;
                mission_last_time = ros::Time::now();
            } 
        }
        break;
        case hover_only:
        {
            pose_pub.pose.position.x = position_H.pose.position.x;
            pose_pub.pose.position.y = position_H.pose.position.y;
            pose_pub.pose.position.z = position_H.pose.position.z;
            if(ros::Time::now() - mission_last_time >ros::Duration(5))
            {
                current_mission_state = land;
            }
        }
        break;
        case land:
        break;

    }
}

/**************************************function definition**************************************/
//Euclidean distance between two point
double distance(double x1, double x2, double y1, double y2, double z1, double z2)
{
    return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1)+(z2-z1)*(z2-z1));
}

// limit angle_rad to [-pi,pi]
float wrap_pi(float angle_rad)
{
    //value is inf or NaN
    //the macro definition of M_PI and NAN are both in <math.h>
    //M_PI means 3.14159265257...,NAN means not define

    int c = 0;

    if (angle_rad > 10 || angle_rad < -10) 
    {
        return angle_rad;
    }
    
    while (angle_rad >= M_PI) 
    {
        angle_rad -= M_PI*2;

        if (c++ > 3) 
        {
            return NAN;
        }
    }
    c = 0;
    while (angle_rad < -M_PI) 
    {
        angle_rad += M_PI*2;

        if (c++ > 3) 
        {
            return NAN;
        }
    }
    return angle_rad;
}