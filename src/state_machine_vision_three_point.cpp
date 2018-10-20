/*************************************************************************
@file           vision_test.cpp
@date           2018/10/02
@author         liuxu
@email          liuxu.ccc@gmail.com
@description    vision test 
                recognize->go->fall->grab->rise->fall->place->rise->move->land
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

#include <state_machine/Vision_Position_Raw.h> 


/***************************function declare****************************/
void state_machine_fun(void);
double Distance_of_Two(double x1, double x2, double y1, double y2, double z1, double z2);

/***************************variable definition*************************/
//position  ENU
geometry_msgs::PoseStamped position_home;
geometry_msgs::PoseStamped position_componnet;
geometry_msgs::PoseStamped position_construction;

geometry_msgs::PoseStamped pose_pub; 
geometry_msgs::TwistStamped vel_take_off;
geometry_msgs::TwistStamped vel_ascend_com;
geometry_msgs::TwistStamped vel_ascend_con;
geometry_msgs::TwistStamped vel_descend;

geometry_msgs::PoseStamped position_grab;
geometry_msgs::PoseStamped position_judge;
geometry_msgs::PoseStamped position_place;
geometry_msgs::PoseStamped position_safe;
geometry_msgs::PoseStamped position_box;

ros::Publisher local_pos_pub;
ros::Publisher local_vel_pub;
ros::Publisher fixed_target_pub;
ros::Publisher vision_position_pub;


//state_machine state
static const int takeoff = 1;
static const int position_H_go = 2;
static const int position_H_hover = 3;
static const int position_Com_go = 4;
static const int hover_to_recognize = 5;
static const int component_locate = 6;
static const int component_hover = 7;
static const int Com_get_close = 8;
static const int Com_get_fit = 9;
static const int component_grab = 10;
static const int Com_leave = 11;
static const int grab_status_judge = 12;
static const int position_Con_go = 13;
static const int position_Con_hover = 14;
static const int place_point_get_close = 15;
static const int place_point_adjust = 16;
static const int component_place = 17;
static const int place_done = 18;
static const int Con_leave = 19;
static const int place_status_judge = 20;
static const int return_home = 21;
static const int land = 22;

//vision recognize fail
static const int vision_fail_process = 23;



//mission 
int current_pos_state = takeoff;
int loop = 0; 

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

//velocity send
bool velocity_control_enable = true;
bool fix_target_receive_enable = true;
bool initial_enable = false;
bool vision_position_receive_enable = false;

/*************************constant defunition***************************/

#define HOME_HEIGHT             3.0
#define OBSERVE_HEIGET          3.0
#define CONSTRUCT_HEIGET        3.0
#define TAKE_OFF_HEIGHT         1.5
#define ASCEND_VELOCITY_CON     0.3
#define ASCEND_VELOCITY_COM     0.6
#define TAKE_OFF_VELOCITY       1.5
#define DESCEND_VELOCITY        0.3
#define RECOGNIZE_HEIGHT        2.5
#define BOX_HEIGET              0.25
#define PLACE_HEIGET            0.7
#define Z_BIAS_AER_BOTT         0.1
#define LOCATE_ACCURACY_HIGH    0.5
#define LOCATE_ACCURACY_GRAB    0.2
#define LOCATE_ACCURACY_ROUGH   1.0

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

state_machine::FIXED_TARGET_POSITION_P2M fix_target_position;
void fixed_target_position_p2m_cb(const state_machine::FIXED_TARGET_POSITION_P2M::ConstPtr& msg)
{
    fix_target_position = *msg;

    if(fix_target_receive_enable ==true)
    {
        fix_target_return.home_x = fix_target_position.home_x;
        fix_target_return.home_y = fix_target_position.home_y;
        fix_target_return.home_z = -HOME_HEIGHT + fix_target_position.home_z;

        fix_target_return.component_x = fix_target_position.component_x;
        fix_target_return.component_y = fix_target_position.component_y;
        fix_target_return.component_z = -OBSERVE_HEIGET + fix_target_position.component_z;

        fix_target_return.construction_x = fix_target_position.construction_x;
        fix_target_return.construction_y = fix_target_position.construction_y;
        fix_target_return.construction_z = -CONSTRUCT_HEIGET + fix_target_position.construction_z;

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
        position_home.pose.position.z = HOME_HEIGHT - fix_target_position.home_z;
        //position of componnet
        position_componnet.pose.position.x = fix_target_position.component_y;
        position_componnet.pose.position.y = fix_target_position.component_x;
        position_componnet.pose.position.z = OBSERVE_HEIGET - fix_target_position.component_z;
        //position of construction
        position_construction.pose.position.x = fix_target_position.construction_y;
        position_construction.pose.position.y = fix_target_position.construction_x;
        position_construction.pose.position.z = CONSTRUCT_HEIGET - fix_target_position.construction_z;

        fix_target_receive_enable = false;
    
        // //adjust angular,face north
        // float yaw_sp=M_PI_2;
        // //home
        // position_home.pose.orientation.x = 0;
        // position_home.pose.orientation.y = 0;
        // position_home.pose.orientation.z = sin(yaw_sp/2);
        // position_home.pose.orientation.w = cos(yaw_sp/2);
        // //componnet
        // position_componnet.pose.orientation.x = 0;
        // position_componnet.pose.orientation.y = 0;
        // position_componnet.pose.orientation.z = sin(yaw_sp/2);
        // position_componnet.pose.orientation.w = cos(yaw_sp/2);
        // //construction
        // position_construction.pose.orientation.x = 0;
        // position_construction.pose.orientation.y = 0;
        // position_construction.pose.orientation.z = sin(yaw_sp/2);
        // position_construction.pose.orientation.w = cos(yaw_sp/2);
    }
}

state_machine::Vision_Position_Raw vision_position_raw;
state_machine::Vision_Position_Raw vision_position_raw_use;
state_machine::VISION_POSITION_GET_M2P vision_position_get;
state_machine::VISION_POSITION_GET_M2P vision_position_m2p;
void vision_position_cb(const state_machine::Vision_Position_Raw::ConstPtr& msg)
{
    vision_position_raw = *msg;

    if (vision_position_receive_enable == true)
    {
        vision_position_raw_use = vision_position_raw;
        vision_position_get.component_position_x = vision_position_raw_use.x;
        vision_position_get.component_position_y = vision_position_raw_use.y;
        vision_position_get.component_position_z = vision_position_raw_use.z;
    }
    
    vision_position_m2p.loop_value = loop;
    vision_position_m2p.component_position_x = vision_position_raw.x;
    vision_position_m2p.component_position_y = vision_position_raw.y;
    vision_position_m2p.component_position_z = vision_position_raw.z;

    vision_position_pub.publish(vision_position_m2p);
}

// state_machine::Distance distance;
// void distance_cb(const state_machine::Distance::ConstPtr& msg)
// {
//     distance = *msg;
//     ROS_INFO("distance:%f",distance.distance);
// }

/*****************************main function*****************************/
int main(int argc, char **argv)
{
    ros::init(argc, argv, "offboard_node");
    ros::NodeHandle nh;

    vel_take_off.twist.linear.x = 0.0f;
    vel_take_off.twist.linear.y = 0.0f;
    vel_take_off.twist.linear.z = TAKE_OFF_VELOCITY;
    vel_take_off.twist.angular.x = 0.0f;
    vel_take_off.twist.angular.y = 0.0f;
    vel_take_off.twist.angular.z = 0.0f;

    vel_ascend_com.twist.linear.x = 0.0f;
    vel_ascend_com.twist.linear.y = 0.0f;
    vel_ascend_com.twist.linear.z = ASCEND_VELOCITY_COM;
    vel_ascend_com.twist.angular.x = 0.0f;
    vel_ascend_com.twist.angular.y = 0.0f;
    vel_ascend_com.twist.angular.z = 0.0f;

    vel_ascend_con.twist.linear.x = 0.0f;
    vel_ascend_con.twist.linear.y = 0.0f;
    vel_ascend_con.twist.linear.z = ASCEND_VELOCITY_CON;
    vel_ascend_con.twist.angular.x = 0.0f;
    vel_ascend_con.twist.angular.y = 0.0f;
    vel_ascend_con.twist.angular.z = 0.0f;

    //descend to grab and place velocity
    vel_descend.twist.linear.x = 0.0f;
    vel_descend.twist.linear.y = 0.0f;
    vel_descend.twist.linear.z = -DESCEND_VELOCITY;
    vel_descend.twist.angular.x = 0.0f;
    vel_descend.twist.angular.y = 0.0f;
    vel_descend.twist.angular.z = 0.0f;

    vision_position_get.component_position_x = 0;
    vision_position_get.component_position_y = 0;
    vision_position_get.component_position_z = 0;

    ros::Subscriber state_sub = nh.subscribe<state_machine::State>("mavros/state",10,state_cb);
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose",10,pose_cb);
    ros::Subscriber task_status_sub = nh.subscribe<state_machine::TASK_STATUS_CHANGE_P2M>("mavros/task_status_change_p2m",10,task_status_change_p2m_cb);
    ros::Subscriber fixed_target_sub = nh.subscribe<state_machine::FIXED_TARGET_POSITION_P2M>("mavros/fixed_target_position_p2m",10,fixed_target_position_p2m_cb);    
    ros::Subscriber vision_position_sub = nh.subscribe<state_machine::Vision_Position_Raw>("vision_position",10,vision_position_cb);
    //ros::Subscriber distance_sub = nh.subscribe<state_machine::Distance>("distance",10,distance_cb);

    local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local",10);
    local_vel_pub = nh.advertise<geometry_msgs::TwistStamped>("mavros/setpoint_velocity/cmd_vel",10);
    fixed_target_pub = nh.advertise<state_machine::FIXED_TARGET_RETURN_M2P>("mavros/fixed_target_return_m2p",10);
    vision_position_pub = nh.advertise<state_machine::VISION_POSITION_GET_M2P>("mavros/vision_position_get_m2p",10);
    ros::Publisher grab_status_pub = nh.advertise<state_machine::GRAB_STATUS_M2P>("mavros/grab_status_m2p",10);
    ros::Publisher task_status_pub = nh.advertise<state_machine::TASK_STATUS_MONITOR_M2P>("mavros/task_status_monitor_m2p",10);

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
		local_vel_pub.publish(vel_take_off);
		ros::spinOnce();
		rate.sleep();
	}

    ROS_INFO("Initialization finished");

	while(ros::ok())
	{
        static bool display_falg = true;
        
        if(current_state.mode == "OFFBOARD" && current_state.armed)
        {
            state_machine_fun();
            ROS_INFO("current_pos_state:%d",current_pos_state);
        }
        else if(velocity_control_enable == true)
        {
            local_vel_pub.publish(vel_take_off);
            if(display_falg == true)
            {
                ROS_INFO("Wait for switch to offboard...");
                display_falg = false;
            }
        }

        if(current_state.armed && current_pos_state == land)
        {
            if(current_state.mode != "MANUAL" && current_state.mode != "AUTO.LAND" && 
              (ros::Time::now() - landing_last_request > ros::Duration(5.0)))
                {
                    if(land_client.call(landing_cmd) && landing_cmd.response.success)
                    {
                        ROS_INFO("AUTO LANDING");
                    }
                    landing_last_request = ros::Time::now();
                }
        }

        task_status_monitor.task_status = current_pos_state;
        task_status_monitor.loop_value = loop;
        task_status_monitor.target_x = position_box.pose.position.y;
        task_status_monitor.target_y = position_box.pose.position.x;
        task_status_monitor.target_z = position_box.pose.position.z;
        task_status_monitor.sensor_distance = 0;
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
            velocity_control_enable = false;
            pose_pub = position_home;
            local_vel_pub.publish(vel_take_off);
            if((current_position.pose.position.z + fix_target_position.home_z) > TAKE_OFF_HEIGHT)
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
            if (Distance_of_Two(current_position.pose.position.x,position_home.pose.position.x,
                                current_position.pose.position.y,position_home.pose.position.y,
                                current_position.pose.position.z,position_home.pose.position.z) < LOCATE_ACCURACY_ROUGH)
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
            if(ros::Time::now() - last_time > ros::Duration(3.0))
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
            if (Distance_of_Two(current_position.pose.position.x,position_componnet.pose.position.x,
                                current_position.pose.position.y,position_componnet.pose.position.y,
                                current_position.pose.position.z,position_componnet.pose.position.z) < LOCATE_ACCURACY_ROUGH)
            {
                initial_enable = true;
                vision_position_receive_enable == true;
                current_pos_state = hover_to_recognize;
                last_time = ros::Time::now();
            }
        }
        break;
        case hover_to_recognize:
        {   
            static float position_x_aver = 0;
            static float position_y_aver = 0;
            static float position_z_aver = 0;
            static int vision_count1 = 11;
            static int vision_lost_count = 0;

            pose_pub = position_componnet;
            local_pos_pub.publish(position_componnet);

            if(vision_position_get.component_position_x != 0 || vision_position_get.component_position_y != 0 || 
               vision_position_get.component_position_z != 0)
            {
                vision_lost_count = 0;

                if(ros::Time::now() - last_time > ros::Duration(1.0) && vision_count1 > 10)
                {
                    vision_count1 = 0;
                }
                if(ros::Time::now() - last_time > ros::Duration(0.5) && vision_count1 < 10)
                {
                    position_x_aver = (position_x_aver * vision_count1 + vision_position_get.component_position_x)/(vision_count1 + 1);
                    position_y_aver = (position_y_aver * vision_count1 + vision_position_get.component_position_y)/(vision_count1 + 1);
                    position_z_aver = (position_z_aver * vision_count1 + vision_position_get.component_position_z)/(vision_count1 + 1);
                    // current_pos_state = component_locate;   //need change Z place to component_locate
                    vision_count1++;
                    //display_enable = true;
                }
                else if(vision_count1 == 10)
                {
                    position_box.pose.position.x = position_y_aver;
                    position_box.pose.position.y = position_x_aver;
                    position_box.pose.position.z = current_position.pose.position.z;

                    position_grab.pose.position.x = position_box.pose.position.x;
                    position_grab.pose.position.y = position_box.pose.position.y;
                    position_grab.pose.position.z = current_position.pose.position.z - position_z_aver + Z_BIAS_AER_BOTT;

                    vision_count1 = 11;

                    position_x_aver = 0;
                    position_y_aver = 0;
                    position_z_aver = 0;

                    vision_position_get.component_position_x = 0;
                    vision_position_get.component_position_y = 0;
                    vision_position_get.component_position_z = 0;

                    vision_position_receive_enable == false;

                    if(abs(current_position.pose.position.x - position_box.pose.position.x) > 5 ||
                       abs(current_position.pose.position.y - position_box.pose.position.y) > 5)
                    {
                        current_pos_state = vision_fail_process;
                        last_time = ros::Time::now();
                        break;
                    }
                    else
                    {
                        current_pos_state = component_locate;
                        last_time = ros::Time::now();
                        break;
                    }
                }  
            }
            else 
            {
                vision_lost_count++;
                if (vision_lost_count > 40)
                {
                    vision_lost_count = 0;
                    current_pos_state = vision_fail_process;
                    last_time = ros::Time::now();
                    break;
                }
            }
            
        }
        break;
        case component_locate:
        {
            pose_pub = position_box;
            local_pos_pub.publish(position_box);
            if (Distance_of_Two(current_position.pose.position.x,position_box.pose.position.x,
                                current_position.pose.position.y,position_box.pose.position.y,
                                current_position.pose.position.z,position_box.pose.position.z) < LOCATE_ACCURACY_HIGH)
            {
                current_pos_state = component_hover;
                last_time = ros::Time::now();
            }
        }
        break;
        // case component_hover:
        // {
        //     static int vision_count = 11;
        //     static int row_count_x_max = 0;
        //     static int row_count_x_min = 0;
        //     static int row_count_y_max = 0;
        //     static int row_count_y_min = 0;
        //     static int row_count_z_max = 0;
        //     static int row_count_z_min = 0;
        //     static float position_x_max = 0;
        //     static float position_y_max = 0;
        //     static float position_x_min = 0;
        //     static float position_y_min = 0;
        //     static float position_z_max = 0;
        //     static float position_z_min = 0;
        //     static float position_x_total = 0;
        //     static float position_y_total = 0;
        //     static float position_z_total = 0;
        //     static float position_x_average = 0;
        //     static float position_y_average = 0;
        //     static float position_z_average = 0;
        //     static float position[10][3] = {0};
            
        //     if (ros::Time::now() - last_time > ros::Duration(1.0) && vision_count > 10)
        //     {
        //         vision_count = 0;
        //     }
        //     if(ros::Time::now() - last_time > ros::Duration(0.5) && vision_count < 10)
        //     {
        //         // position_x_average = (position_x_average * vision_count + vision_position.component_position_x)/(vision_count);
        //         // position_y_average = (position_x_average * vision_count + vision_position.component_position_y)/(vision_count);
                
        //         position[vision_count][0] = vision_position_get.component_position_x;
        //         position[vision_count][1] = vision_position_get.component_position_y;
        //         position[vision_count][2] = vision_position_get.component_position_z;

        //         if(initial_enable == true)
        //         {
        //             position_x_max = position[0][0];
        //             position_y_max = position[0][1];
        //             position_x_min = position[0][0];
        //             position_y_min = position[0][1]; 
        //             position_z_max = position[0][2];
        //             position_z_min = position[0][2];                   
        //             initial_enable = false;
        //         }

        //         if(vision_position_get.component_position_x > position_x_max)
        //         {
        //             position_x_max = vision_position_get.component_position_x;
        //             row_count_x_max = vision_count;
        //         }
        //         if(vision_position_get.component_position_y > position_y_max)
        //         {
        //             position_y_max = vision_position_get.component_position_y;
        //             row_count_y_max = vision_count;
        //         }
        //         if(vision_position_get.component_position_x < position_x_min)
        //         {
        //             position_x_min = vision_position_get.component_position_x;
        //             row_count_x_min = vision_count;
        //         }
        //         if(vision_position_get.component_position_y < position_y_min)
        //         {
        //             position_y_min = vision_position_get.component_position_y;
        //             row_count_y_min = vision_count;
        //         }
        //         if(vision_position_get.component_position_z > position_z_max)
        //         {
        //             position_z_max = vision_position_get.component_position_z;
        //             row_count_z_max = vision_count;
        //         }
        //         if(vision_position_get.component_position_z < position_z_min)
        //         {
        //             position_z_min = vision_position_get.component_position_z;
        //             row_count_z_min = vision_count;
        //         }
                 
        //         vision_count++;
        //     }
        //     else if(vision_count == 10)
        //     {

        //         for(int i = 0;i < 10;i++)
        //         {
        //             if(i != row_count_x_max && i != row_count_x_min)
        //             {
        //                 position_x_total += position[i][0]; 
        //                 //ROS_INFO("position[%d][0]:%f",i,position_by_calculated.pose.position.x);
        //             }
                           
        //         }
        //         for(int i = 0;i < 10;i++)
        //         {
        //             if(i != row_count_y_max && i != row_count_y_min)
        //             {
        //                 position_y_total += position[i][1];
        //                 //ROS_INFO("position[%d][1]:%f",i,position_by_calculated.pose.position.x);
        //             }
        //         }
        //         for(int i = 0;i < 10;i++)
        //         {
        //             if(i != row_count_z_max && i != row_count_z_min)
        //             {
        //                 position_z_total += position[i][2];
        //                 //ROS_INFO("position[%d][1]:%f",i,position_by_calculated.pose.position.x);
        //             }
        //         }
        //         // ROS_INFO("position_x_max:%f",position_x_max);
        //         // ROS_INFO("row_count_x_max:%d",row_count_x_max);
        //         // ROS_INFO("position_y_max:%f",position_y_max);
        //         // ROS_INFO("row_count_y_max:%d",row_count_y_max);
        //         // ROS_INFO("position_x_min:%f",position_x_min);
        //         // ROS_INFO("row_count_x_min:%d",row_count_x_min);
        //         // ROS_INFO("position_y_min:%f",position_y_min);
        //         // ROS_INFO("row_count_y_min:%d",row_count_y_min);

        //         position_x_average = position_x_total/8;
        //         position_y_average = position_y_total/8;
        //         position_z_average = position_z_total/8;
                
        //         //change to ENU
        //         position_grab.pose.position.x = position_y_average;
        //         position_grab.pose.position.y = position_x_average;
        //         position_grab.pose.position.z = -position_z_average + Z_BIAS_AER_BOTT;

        //         // ROS_INFO("position_by_calculated.x:%f",position_by_calculated.pose.position.x);
        //         // ROS_INFO("position_by_calculated.y:%f",position_by_calculated.pose.position.y);
        //         // ROS_INFO("position_by_calculated.z:%f",position_by_calculated.pose.position.z);

        //         vision_count = 11;
        //         row_count_x_max = 0;
        //         row_count_x_min = 0;
        //         row_count_y_max = 0;
        //         row_count_y_min = 0;
        //         row_count_z_max = 0;
        //         row_count_z_min = 0;
        //         position_x_average = 0;
        //         position_y_average = 0;
        //         position_z_average = 0;
        //         position_x_max = 0;
        //         position_y_max = 0;
        //         position_x_min = 0;
        //         position_y_min = 0;
        //         position_z_max = 0;
        //         position_z_min = 0;
        //         position_x_total = 0;
        //         position_y_total = 0;
        //         position_z_total = 0;
        //         position[10][3] = {0};
        //         current_pos_state = Com_get_close;   //need change Z place to component_locate
        //         last_time = ros::Time::now();
        //         //display_enable = true;
        //     }
        // }
        // break;
        case component_hover:
        {
            static int accuracy_count1 = 0;  //for improve accuracy
            static int hover_count1 = 0;
            pose_pub = position_componnet;
            local_pos_pub.publish(position_componnet);
            if(ros::Time::now() - last_time > ros::Duration(2.0) && hover_count1 == 0)
            {
                hover_count1++;
            }
            if (ros::Time::now() - last_time > ros::Duration(0.5) && hover_count1 > 0)
            {
                if (Distance_of_Two(current_position.pose.position.x,position_componnet.pose.position.x,
                                    current_position.pose.position.y,position_componnet.pose.position.y,
                                    current_position.pose.position.z,position_componnet.pose.position.z) < 0.15)
                {
                    accuracy_count1++;
                    if(accuracy_count1 > 3)
                    {
                        current_pos_state = Com_get_close;
                        accuracy_count1 = 0;
                        hover_count1 = 0;
                        last_time = ros::Time::now();
                        break;
                    }
                }
                else
                {
                    accuracy_count1 = 0;
                }
            }           
            hover_count1++;
            if(hover_count1 > 15)
            {
                current_pos_state = Com_get_close;
                accuracy_count1 = 0;
                hover_count1 = 0;
                last_time = ros::Time::now();
                break;
            }
        }
        break;
        case Com_get_close:
        {
            local_pos_pub.publish(position_grab);
            pose_pub = position_grab;
            if (Distance_of_Two(current_position.pose.position.x,position_grab.pose.position.x,
                                current_position.pose.position.y,position_grab.pose.position.y,
                                current_position.pose.position.z,position_grab.pose.position.z) < LOCATE_ACCURACY_GRAB)
            {
                current_pos_state = component_grab;
                last_time = ros::Time::now();
            }
        }
        break;
        case component_grab:
        {
            static int grab_count = 0;
            pose_pub = position_grab;
            local_pos_pub.publish(position_grab);
            if(ros::Time::now() - last_time > ros::Duration(1.0) && grab_count == 0)
            {
                grab_count++;
            }
            if(ros::Time::now() - last_time > ros::Duration(0.5) && grab_count > 0)
            {
                grab_count++;
                if(grab_count > 30)
                {
                    grab_count = 0;
                    current_pos_state = Com_leave;
                    last_time = ros::Time::now();
                    break;
                }
            }
        }
        break;
        case Com_leave:
        {
            pose_pub = position_componnet;
            local_vel_pub.publish(vel_ascend_com);
            if ((current_position.pose.position.z + fix_target_position.component_z) > 3)
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
            pose_pub = position_judge;
            local_pos_pub.publish(position_judge);
            if(ros::Time::now() - last_time > ros::Duration(5.0))
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
            if (Distance_of_Two(current_position.pose.position.x,position_construction.pose.position.x,
                                current_position.pose.position.y,position_construction.pose.position.y,
                                current_position.pose.position.z,position_construction.pose.position.z) < LOCATE_ACCURACY_HIGH)
            {
                position_place.pose.position.x = position_construction.pose.position.x;
                position_place.pose.position.y = position_construction.pose.position.y;
                position_place.pose.position.z = -fix_target_position.construction_z + loop * BOX_HEIGET + PLACE_HEIGET;

                current_pos_state = position_Con_hover;
                last_time = ros::Time::now();
            }
        }
        break;
        case position_Con_hover:
        {
            static int accuracy_count2 = 0;  //for improve accuracy
            static int hover_count2 = 0;
            pose_pub = position_construction;
            local_pos_pub.publish(position_construction);
            if(ros::Time::now() - last_time > ros::Duration(2.0) && hover_count2 == 0)
            {
                hover_count2++;
            }
            if (ros::Time::now() - last_time > ros::Duration(0.5) && hover_count2 > 0)
            {
                if (Distance_of_Two(current_position.pose.position.x,position_construction.pose.position.x,
                                    current_position.pose.position.y,position_construction.pose.position.y,
                                    current_position.pose.position.z,position_construction.pose.position.z) < 0.15)
                {
                    accuracy_count2++;
                    if(accuracy_count2 > 3)
                    {
                        current_pos_state = place_point_get_close;
                        accuracy_count2 = 0;
                        hover_count2 = 0;
                        last_time = ros::Time::now();
                        break;
                    }
                }
                else
                {
                    accuracy_count2 = 0;
                }
            }           
            hover_count2++;
            if(hover_count2 > 10)
            {
                current_pos_state = place_point_get_close;
                accuracy_count2 = 0;
                hover_count2 = 0;
                last_time = ros::Time::now();
                break;
            }
        }
        break;
        case place_point_get_close:
        {
            pose_pub = position_place;
            local_pos_pub.publish(position_place);
            if (Distance_of_Two(current_position.pose.position.x,position_place.pose.position.x,
                                current_position.pose.position.y,position_place.pose.position.y,
                                current_position.pose.position.z,position_place.pose.position.z) < LOCATE_ACCURACY_HIGH)
            {
                current_pos_state = place_point_adjust;
                last_time = ros::Time::now();
            }
        }
        break;
        case place_point_adjust:
        {
            static int accuracy_count3 = 0;  //for improve accuracy
            static int hover_count3 = 0;
            pose_pub = position_place;
            local_pos_pub.publish(position_place);
            if(ros::Time::now() - last_time > ros::Duration(2.0) && hover_count3 == 0)
            {
                hover_count3++;
            }
            if (ros::Time::now() - last_time > ros::Duration(0.5) && hover_count3 > 0)
            {
                if (Distance_of_Two(current_position.pose.position.x,position_place.pose.position.x,
                                    current_position.pose.position.y,position_place.pose.position.y,
                                    current_position.pose.position.z,position_place.pose.position.z) < 0.15)
                {
                    accuracy_count3++;
                    if(accuracy_count3 > 3)
                    {
                        current_pos_state = component_place;
                        accuracy_count3 = 0;
                        hover_count3 = 0;
                        last_time = ros::Time::now();
                        break;
                    }
                }
                else
                {
                    accuracy_count3 = 0;
                }
            }           
            hover_count3++;
            if(hover_count3 > 10)
            {
                current_pos_state = component_place;
                accuracy_count3 = 0;
                hover_count3 = 0;
                last_time = ros::Time::now();
                break;
            }
        }
        break;
        case component_place:
        {
            static int place_count = 0;
            pose_pub = position_place;
            local_pos_pub.publish(position_place);
            if(ros::Time::now() - last_time > ros::Duration(1.0) && place_count == 0)
            {
                place_count++;
            }
            if(ros::Time::now() - last_time > ros::Duration(0.5) && place_count > 0)
            {
                place_count++;
                if(place_count > 20)
                {
                    place_count = 0;
                    current_pos_state = Con_leave;
                    last_time = ros::Time::now();
                    break;
                }
            }
        }
        break;
        case Con_leave:
        {
            pose_pub = position_place;
            local_vel_pub.publish(vel_ascend_con);
            if ((current_position.pose.position.z + fix_target_position.construction_z) > (3 + BOX_HEIGET*loop))
            {
                position_safe.pose.position.x = current_position.pose.position.x;
                position_safe.pose.position.y = current_position.pose.position.y;
                position_safe.pose.position.z = current_position.pose.position.z;
                current_pos_state = place_status_judge;
                last_time = ros::Time::now();
            }
        }
        break;
        case place_status_judge:
        {
            pose_pub = position_safe;
            local_pos_pub.publish(position_safe);
            if(ros::Time::now() - last_time > ros::Duration(5.0))
            {
                current_pos_state = return_home;
                last_time = ros::Time::now();
            }
        }
        break;
        case vision_fail_process:
        {
            current_pos_state = return_home;
            last_time = ros::Time::now();
        }
        break;
        case  return_home:
        {
            pose_pub = position_home;
            local_pos_pub.publish(position_home);
            if (Distance_of_Two(current_position.pose.position.x,position_home.pose.position.x,
                                current_position.pose.position.y,position_home.pose.position.y,
                                current_position.pose.position.z,position_home.pose.position.z) < LOCATE_ACCURACY_HIGH)
            {
                current_pos_state = land;
                last_time = ros::Time::now();
            }
        }
        break;
        case land:
        break;
    }
}

/**************************************function definition**************************************/

double Distance_of_Two(double x1, double x2, double y1, double y2, double z1, double z2)
{
    return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1)+(z2-z1)*(z2-z1));
}