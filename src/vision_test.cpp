/*************************************************************************
@file           vision_test.cpp
@date           2018/10/02
@author         liuxu
@email          liuxu.ccc@gmail.com
@description    vision test 
                recognize->go->fall->grab->rise->fall->place->rise->move->land
*************************************************************************/

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <state_machine/CommandTOL.h>
#include <state_machine/State.h>
#include "math.h"

#include <state_machine/TASK_STATUS_CHANGE_P2M.h>
#include <state_machine/VISION_POSITION_GET_M2P.h>
#include <state_machine/Vision_Position_Raw.h> 

#include <state_machine/Distance.h> 

/***************************function declare****************************/
void state_machine_fun(void);

/***************************variable definition*************************/

geometry_msgs::PoseStamped pose_pub;  //ENU
geometry_msgs::TwistStamped vel_pub;  //ENU
geometry_msgs::TwistStamped vel_descend;

geometry_msgs::PoseStamped position_grab;
geometry_msgs::PoseStamped position_judge;
geometry_msgs::PoseStamped position_place;
geometry_msgs::PoseStamped position_safe;

ros::Publisher vision_position_pub;

ros::Publisher local_pos_pub;
ros::Publisher local_vel_pub;


//state_machine state
static const int takeoff = 1;
static const int target_go = 2;
static const int hover_to_recognize = 3;
static const int component_locate = 4;
static const int component_hover = 5;
static const int component_get_close = 6;
static const int component_grab = 7;
static const int component_leave = 8;
static const int grab_status_judge = 9;
static const int place_point_get_close = 12;
static const int componnet_place = 13;
static const int place_done = 14;
static const int Con_leave = 15;
static const int Con_hover = 16;
static const int go_safe = 17;
static const int land = 18;


//mission 
int current_pos_state = takeoff;
int loop = 0; 

//time
ros::Time last_time;

//land
state_machine::CommandTOL landing_cmd;
ros::ServiceClient land_client;
ros::Time landing_last_request;

int vision_count = 0;
float position_x_average = 0;
float position_y_average = 0;

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

state_machine::TASK_STATUS_CHANGE_P2M task_status_change;
void task_status_change_p2m_cb(const state_machine::TASK_STATUS_CHANGE_P2M::ConstPtr& msg)
{
    task_status_change = *msg;
}

state_machine::Vision_Position_Raw vision_position_raw;
state_machine::VISION_POSITION_GET_M2P vision_position;
void vision_position_cb(const state_machine::Vision_Position_Raw::ConstPtr& msg)
{
    vision_position_raw = *msg;

    vision_position.loop_value = loop;
    vision_position.component_position_x = vision_position_raw.x;
    vision_position.component_position_y = vision_position_raw.y;
    vision_position.component_position_z = vision_position_raw.z;

    vision_position_pub.publish(vision_position);
}

state_machine::Distance distance;
void distance_cb(const state_machine::Distance::ConstPtr& msg)
{
    distance = *msg;
    ROS_INFO("distance:%f",distance.distance);
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

    //descend to grab and place velocity
    vel_descend.twist.linear.x = 0.0f;
    vel_descend.twist.linear.y = 0.0f;
    vel_descend.twist.linear.z = -0.3f;
    vel_descend.twist.angular.x = 0.0f;
    vel_descend.twist.angular.y = 0.0f;
    vel_descend.twist.angular.z = 0.0f;

    pose_pub.pose.position.x = 0;
    pose_pub.pose.position.y = 0;
    pose_pub.pose.position.z = 2.5;

    ros::Subscriber state_sub = nh.subscribe<state_machine::State>("mavros/state",10,state_cb);
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose",10,pose_cb);
    ros::Subscriber task_status_sub = nh.subscribe<state_machine::TASK_STATUS_CHANGE_P2M>("mavros/task_status_change_p2m",10,task_status_change_p2m_cb);
    ros::Subscriber vision_position_sub = nh.subscribe<state_machine::Vision_Position_Raw>("vision_position",10,vision_position_cb);
    ros::Subscriber distance_sub = nh.subscribe<state_machine::Distance>("distance",10,distance_cb);

    local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local",10);
    local_vel_pub = nh.advertise<geometry_msgs::TwistStamped>("mavros/setpoint_velocity/cmd_vel",10);
    vision_position_pub = nh.advertise<state_machine::VISION_POSITION_GET_M2P>("mavros/vision_position_get_m2p",10);
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
            if(current_position.pose.position.z > 1.5)
            {
                current_pos_state = target_go;
                last_time = ros::Time::now();
            }
        }
        break;
        case target_go:
        {
            local_pos_pub.publish(pose_pub);
            if(abs(current_position.pose.position.x - pose_pub.pose.position.x) < 0.2 &&
               abs(current_position.pose.position.y - pose_pub.pose.position.y) < 0.2 &&
               abs(current_position.pose.position.z - pose_pub.pose.position.z) < 0.2 )
            {
                current_pos_state = hover_to_recognize;
                last_time = ros::Time::now();
            }
        }
        break;
        case hover_to_recognize:
        {
            local_pos_pub.publish(pose_pub);
            
            if(ros::Time::now() - last_time < ros::Duration(3.0))
            {
                position_x_average = (position_x_average * vision_count + vision_position.component_position_x)/(vision_count++);
                position_y_average = (position_x_average * vision_count + vision_position.component_position_y)/(vision_count++);
            }
            else
            {
                pose_pub.pose.position.x = position_x_average;
                pose_pub.pose.position.y = position_y_average;
                pose_pub.pose.position.z = 2.5;
                vision_count = 0;
                position_x_average = 0;
                position_y_average = 0;
                current_pos_state = component_locate;
                last_time = ros::Time::now();
            }
        }
        break;
        case component_locate:
        {
            local_pos_pub.publish(pose_pub);
            if(abs(current_position.pose.position.x - pose_pub.pose.position.x) < 0.2 &&
               abs(current_position.pose.position.y - pose_pub.pose.position.y) < 0.2 &&
               abs(current_position.pose.position.z - pose_pub.pose.position.z) < 0.2 )
            {
                current_pos_state = component_hover;
                last_time = ros::Time::now();
            }

        }
        break;
        case component_hover:
        {
            local_pos_pub.publish(pose_pub);
            if(ros::Time::now() - last_time < ros::Duration(3.0))
            {
                current_pos_state = component_get_close;
                last_time = ros::Time::now();
            }
        }
        break;
        case component_get_close:
        {
            local_vel_pub.publish(vel_descend);
            if (distance.distance < 0.5)
            {
                current_pos_state = component_grab;

                position_grab.pose.position.x = current_position.pose.position.x;
                position_grab.pose.position.y = current_position.pose.position.y;
                position_grab.pose.position.z = current_position.pose.position.z;

                last_time = ros::Time::now();
            }
        }
        break;
        case component_grab:
        {
            local_pos_pub.publish(position_grab);
            if(ros::Time::now() - last_time > ros::Duration(5.0))
            {
                current_pos_state = component_leave;
                last_time = ros::Time::now();
            }
        }
        break;
        case component_leave:
        {
            local_vel_pub.publish(vel_pub);
            if (current_position.pose.position.z > 3 )
            {
                position_judge.pose.position.x = current_position.pose.position.x;
                position_judge.pose.position.y = current_position.pose.position.y;
                position_judge.pose.position.z = current_position.pose.position.z;
                current_pos_state = grab_status_judge;
                last_time = ros::Time::now();
            }
        }
        break;
        case grab_status_judge:
        {
            local_pos_pub.publish(position_judge);
            if (ros::Time::now() - last_time > ros::Duration(3.0))
            {
                if (distance.distance < 0.5)
                    current_pos_state = place_point_get_close;
                else
                    current_pos_state = component_get_close;

                last_time = ros::Time::now();
            }
        }
        break;
        case place_point_get_close:
        {
            local_vel_pub.publish(vel_descend);
            if (current_position.pose.position.z  < 0.8 )
            {
                position_place.pose.position.x = current_position.pose.position.x;
                position_place.pose.position.y = current_position.pose.position.y;
                position_place.pose.position.z = current_position.pose.position.z;
                current_pos_state = componnet_place;
                last_time = ros::Time::now();
            }
        }
        break;
        case componnet_place:
        {
            local_pos_pub.publish(position_place);
            if(ros::Time::now() - last_time > ros::Duration(10.0))
            {
                current_pos_state = Con_leave;
                loop = loop +1;
                last_time = ros::Time::now();
            }
        }
        break;
        case Con_leave:
        {
            local_vel_pub.publish(vel_pub);
            if (current_position.pose.position.z > 3 )
            {
                position_safe.pose.position.x = current_position.pose.position.x;
                position_safe.pose.position.y = current_position.pose.position.y;
                position_safe.pose.position.z = current_position.pose.position.z;
                current_pos_state = Con_hover;
                last_time = ros::Time::now();
            }
        }
        break;
        case Con_hover:
        {
            local_pos_pub.publish(position_safe);
            if(ros::Time::now() - last_time > ros::Duration(3.0))
            {
                position_safe.pose.position.x += 2; 
                current_pos_state = go_safe;
                last_time = ros::Time::now();
            }
        }
        break;
        case  go_safe:
        {
            local_pos_pub.publish(position_safe);
            if (abs(current_position.pose.position.x - position_safe.pose.position.x) < 0.5 &&
                abs(current_position.pose.position.y - position_safe.pose.position.y) < 0.5 &&
                abs(current_position.pose.position.z - position_safe.pose.position.z) < 0.5 )
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