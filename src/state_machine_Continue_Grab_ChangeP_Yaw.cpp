/*************************************************************************
@file           state_machine_v1.0.cpp
@date           2018/09/24 11:01
@author         liuxu
@email          liuxu.ccc@gmail.com
@description    fixed 3 position from GCS
*************************************************************************/


/****************************header files********************************/
//system
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <state_machine/CommandTOL.h>
#include <state_machine/State.h>
#include "math.h"

//custom gcs->pix->mavros
#include <state_machine/FIXED_TARGET_POSITION_P2M.h>
#include <state_machine/TASK_STATUS_CHANGE_P2M.h>
//#include <state_machine/FIXED_BOX_POSITION_P2M.h>

//custom mavros->pix->gcs
#include <state_machine/TASK_STATUS_MONITOR_M2P.h>
#include <state_machine/VISION_POSITION_GET_M2P.h>
#include <state_machine/YAW_SP_CALCULATED_M2P.h>
#include <state_machine/FIXED_TARGET_RETURN_M2P.h>
//#include <state_machine/FIXED_BOX_RETURN_M2P.h>
#include <state_machine/GRAB_STATUS_M2P.h>

#include <state_machine/Distance.h> 

/***************************function declare****************************/

void state_machine_fun(void);
double Distance_of_Two(double x1, double x2, double y1, double y2, double z1, double z2);
float wrap_pi(float angle_rad);

/*************************constant defunition***************************/

#define HOME_HEIGHT             5.0
#define ASCEND_VELOCITY_CON     0.3
#define ASCEND_VELOCITY_COM     0.6
#define TAKE_OFF_VELOCITY       1.5
#define DESCEND_VELOCITY        0.3
#define OBSERVE_HEIGET          5.0
#define CONSTRUCT_HEIGET        5.0
#define BOX_HEIGET              0.25
#define PLACE_HEIGET            0.25
#define LOCATE_ACCURACY_HIGH    0.5
#define LOCATE_ACCURACY_GRAB    0.2
#define LOCATE_ACCURACY_ROUGH   1.0
#define GRAB_HEIGHT_MARGIN      0.02
#define BOX_LOOP_MAX            3

/***************************variable definition*************************/
//position  ENU
geometry_msgs::PoseStamped position_home;
geometry_msgs::PoseStamped position_component;
geometry_msgs::PoseStamped position_construction;
geometry_msgs::PoseStamped position_grab;
geometry_msgs::PoseStamped position_place;
geometry_msgs::PoseStamped position_judge;
geometry_msgs::PoseStamped position_of_safe;

//fixed box position
geometry_msgs::PoseStamped position_box1;
geometry_msgs::PoseStamped position_box2;
geometry_msgs::PoseStamped position_box3;
geometry_msgs::PoseStamped position_box4;
geometry_msgs::PoseStamped position_box5;
geometry_msgs::PoseStamped position_box0;

//topic
geometry_msgs::PoseStamped pose_pub; 
geometry_msgs::TwistStamped vel_take_off;
geometry_msgs::TwistStamped vel_ascend_com;
geometry_msgs::TwistStamped vel_ascend_con;
geometry_msgs::TwistStamped vel_descend;

ros::Publisher local_pos_pub;
ros::Publisher local_vel_pub;
ros::Publisher fixed_target_pub;
//ros::Publisher fixed_box_pub;

//bool velocity_control_enable = true;

//state_machine state,every state need target position
static const int takeoff = 1;
static const int position_H_go = 2;
static const int position_H_hover = 3;
static const int position_Com_go = 4;
static const int position_Com_hover = 5;
static const int Com_get_close = 6;
static const int component_grab = 10;
static const int Com_leave = 11;
static const int grab_status_judge = 12;
static const int position_Con_go = 13;
static const int position_Con_hover = 14;
static const int place_point_get_close = 15;
static const int place_point_adjust = 16;
static const int componnet_place = 17;
static const int place_done = 18;
static const int Con_leave = 19;
static const int place_status_judge = 20;
static const int return_home = 21;
static const int land = 22;

//mission 
int grab_loop = 0;
int place_loop = 0;
int current_pos_state = takeoff;

//box_loop
int box_loop = 0;

//time
ros::Time last_time;

//land
state_machine::CommandTOL landing_cmd;
ros::ServiceClient land_client;
ros::Time landing_last_request;

//message to pix
state_machine::FIXED_TARGET_RETURN_M2P fix_target_return;
//state_machine::FIXED_BOX_RETURN_M2P fix_box_return;
state_machine::GRAB_STATUS_M2P grab_status;
state_machine::TASK_STATUS_MONITOR_M2P task_status_monitor;

//velocity send
bool velocity_control_enable = true;
bool fix_target_receive_enable = true;
bool fix_box_receive_enable = true;

//yaw set
float yaw_sp;

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

        fix_target_return.box1_x = fix_target_position.box1_x;
        fix_target_return.box1_y = fix_target_position.box1_y;
        fix_target_return.box1_z = -OBSERVE_HEIGET + fix_target_position.box1_z;

        fix_target_return.box2_x = fix_target_position.box2_x;
        fix_target_return.box2_y = fix_target_position.box2_y;
        fix_target_return.box2_z = -OBSERVE_HEIGET + fix_target_position.box2_z;

        fix_target_return.box3_x = fix_target_position.box3_x;
        fix_target_return.box3_y = fix_target_position.box3_y;
        fix_target_return.box3_z = -OBSERVE_HEIGET + fix_target_position.box3_z;

        fix_target_return.box4_x = fix_target_position.box4_x;
        fix_target_return.box4_y = fix_target_position.box4_y;
        fix_target_return.box4_z = -OBSERVE_HEIGET + fix_target_position.box4_z;

        fix_target_return.box5_x = fix_target_position.box5_x;
        fix_target_return.box5_y = fix_target_position.box5_y;
        fix_target_return.box5_z = -OBSERVE_HEIGET + fix_target_position.box5_z;

        // fix_target_return.box6_x = fix_target_position.component_x;
        // fix_target_return.box6_y = fix_target_position.component_y;
        // fix_target_return.box6_z = -OBSERVE_HEIGET + fix_target_position.component_z;

        fixed_target_pub.publish(fix_target_return);

        ROS_INFO("fix_target_return.home_x:%f",fix_target_return.home_x);
        ROS_INFO("fix_target_return.home_y:%f",fix_target_return.home_y);
        ROS_INFO("fix_target_return.home_z:%f",fix_target_return.home_z);

        ROS_INFO("fix_target_return.component_x:%f",fix_target_return.component_x);
        ROS_INFO("fix_target_return.component_y:%f",fix_target_return.component_y);
        ROS_INFO("fix_target_return.component_z:%f",fix_target_return.component_z);

        ROS_INFO("fix_target_return.construction_x:%f",fix_target_return.construction_x);
        ROS_INFO("fix_target_return.construction_y:%f",fix_target_return.construction_y);
        ROS_INFO("fix_target_return.construction_z:%f",fix_target_return.construction_z);

        ROS_INFO("fix_target_return.box1_x:%f",fix_target_return.box1_x);
        ROS_INFO("fix_target_return.box1_y:%f",fix_target_return.box1_y);
        ROS_INFO("fix_target_return.box1_z:%f",fix_target_return.box1_z);

        ROS_INFO("fix_target_return.box2_x:%f",fix_target_return.box2_x);
        ROS_INFO("fix_target_return.box2_y:%f",fix_target_return.box2_y);
        ROS_INFO("fix_target_return.box2_z:%f",fix_target_return.box2_z);

        ROS_INFO("fix_target_return.box3_x:%f",fix_target_return.box3_x);
        ROS_INFO("fix_target_return.box3_y:%f",fix_target_return.box3_y);
        ROS_INFO("fix_target_return.box3_z:%f",fix_target_return.box3_z);

        ROS_INFO("fix_target_return.box4_x:%f",fix_target_return.box4_x);
        ROS_INFO("fix_target_return.box4_y:%f",fix_target_return.box4_y);
        ROS_INFO("fix_target_return.box4_z:%f",fix_target_return.box4_z);

        ROS_INFO("fix_target_return.box5_x:%f",fix_target_return.box5_x);
        ROS_INFO("fix_target_return.box5_y:%f",fix_target_return.box5_y);
        ROS_INFO("fix_target_return.box5_z:%f",fix_target_return.box5_z);

        //fixed_target_pub.publish(fix_target_return);

        //Coordinate transformation
        //position of home
        position_home.pose.position.x = fix_target_position.home_y;
        position_home.pose.position.y = fix_target_position.home_x;
        position_home.pose.position.z = HOME_HEIGHT - fix_target_position.home_z;
        //position of componnet
        // position_component.pose.position.x = fix_target_position.component_y;
        // position_component.pose.position.y = fix_target_position.component_x;
        // position_component.pose.position.z = OBSERVE_HEIGET - fix_target_position.component_z;
        //position of construction
        position_construction.pose.position.x = fix_target_position.construction_y;
        position_construction.pose.position.y = fix_target_position.construction_x;
        position_construction.pose.position.z = CONSTRUCT_HEIGET - fix_target_position.construction_z;

        position_box1.pose.position.x = fix_target_position.box1_y;
        position_box1.pose.position.y = fix_target_position.box1_x;
        position_box1.pose.position.z = OBSERVE_HEIGET - fix_target_position.box1_z;

        position_box2.pose.position.x = fix_target_position.box2_y;
        position_box2.pose.position.y = fix_target_position.box2_x;
        position_box2.pose.position.z = OBSERVE_HEIGET - fix_target_position.box2_z;

        position_box3.pose.position.x = fix_target_position.box3_y;
        position_box3.pose.position.y = fix_target_position.box3_x;
        position_box3.pose.position.z = OBSERVE_HEIGET - fix_target_position.box3_z;

        position_box4.pose.position.x = fix_target_position.box4_y;
        position_box4.pose.position.y = fix_target_position.box4_x;
        position_box4.pose.position.z = OBSERVE_HEIGET - fix_target_position.box4_z;

        position_box5.pose.position.x = fix_target_position.box5_y;
        position_box5.pose.position.y = fix_target_position.box5_x;
        position_box5.pose.position.z = OBSERVE_HEIGET - fix_target_position.box5_z;

        position_box0.pose.position.x = fix_target_position.component_y;
        position_box0.pose.position.y = fix_target_position.component_x;
        position_box0.pose.position.z = OBSERVE_HEIGET - fix_target_position.component_z;

        yaw_sp = wrap_pi(M_PI_2 - fix_target_return.component_yaw_sp * M_PI/180);

        //home
        position_home.pose.orientation.x = 0;
        position_home.pose.orientation.y = 0;
        position_home.pose.orientation.z = sin(yaw_sp/2);
        position_home.pose.orientation.w = cos(yaw_sp/2);
        //componnet
        position_component.pose.orientation.x = 0;
        position_component.pose.orientation.y = 0;
        position_component.pose.orientation.z = sin(yaw_sp/2);
        position_component.pose.orientation.w = cos(yaw_sp/2);
        //construction
        position_construction.pose.orientation.x = 0;
        position_construction.pose.orientation.y = 0;
        position_construction.pose.orientation.z = sin(yaw_sp/2);
        position_construction.pose.orientation.w = cos(yaw_sp/2);

        //grab
        position_grab.pose.orientation.x = 0;
        position_grab.pose.orientation.y = 0;
        position_grab.pose.orientation.z = sin(yaw_sp/2);
        position_grab.pose.orientation.w = cos(yaw_sp/2);
        //place
        position_place.pose.orientation.x = 0;
        position_place.pose.orientation.y = 0;
        position_place.pose.orientation.z = sin(yaw_sp/2);
        position_place.pose.orientation.w = cos(yaw_sp/2);
        //judge
        position_judge.pose.orientation.x = 0;
        position_judge.pose.orientation.y = 0;
        position_judge.pose.orientation.z = sin(yaw_sp/2);
        position_judge.pose.orientation.w = cos(yaw_sp/2);
        //safe
        position_of_safe.pose.orientation.x = 0;
        position_of_safe.pose.orientation.y = 0;
        position_of_safe.pose.orientation.z = sin(yaw_sp/2);
        position_of_safe.pose.orientation.w = cos(yaw_sp/2);

        // //box0
        // position_box0.pose.orientation.x = 0;
        // position_box0.pose.orientation.y = 0;
        // position_box0.pose.orientation.z = sin(yaw_sp/2);
        // position_box0.pose.orientation.w = cos(yaw_sp/2);
        // //box1
        // position_box1.pose.orientation.x = 0;
        // position_box1.pose.orientation.y = 0;
        // position_box1.pose.orientation.z = sin(yaw_sp/2);
        // position_box1.pose.orientation.w = cos(yaw_sp/2);
        // //box2
        // position_box2.pose.orientation.x = 0;
        // position_box2.pose.orientation.y = 0;
        // position_box2.pose.orientation.z = sin(yaw_sp/2);
        // position_box2.pose.orientation.w = cos(yaw_sp/2);
        // //box3
        // position_box3.pose.orientation.x = 0;
        // position_box3.pose.orientation.y = 0;
        // position_box3.pose.orientation.z = sin(yaw_sp/2);
        // position_box3.pose.orientation.w = cos(yaw_sp/2);
        // //box4
        // position_box4.pose.orientation.x = 0;
        // position_box4.pose.orientation.y = 0;
        // position_box4.pose.orientation.z = sin(yaw_sp/2);
        // position_box4.pose.orientation.w = cos(yaw_sp/2);
        // //box5
        // position_box5.pose.orientation.x = 0;
        // position_box5.pose.orientation.y = 0;
        // position_box5.pose.orientation.z = sin(yaw_sp/2);
        // position_box5.pose.orientation.w = cos(yaw_sp/2);


        fix_target_receive_enable = false;
    
        // //adjust angular,face north
        // float yaw_sp=M_PI_2;
        // //home
        // position_home.pose.orientation.x = 0;
        // position_home.pose.orientation.y = 0;
        // position_home.pose.orientation.z = sin(yaw_sp/2);
        // position_home.pose.orientation.w = cos(yaw_sp/2);
        // //componnet
        // position_component.pose.orientation.x = 0;
        // position_component.pose.orientation.y = 0;
        // position_component.pose.orientation.z = sin(yaw_sp/2);
        // position_component.pose.orientation.w = cos(yaw_sp/2);
        // //construction
        // position_construction.pose.orientation.x = 0;
        // position_construction.pose.orientation.y = 0;
        // position_construction.pose.orientation.z = sin(yaw_sp/2);
        // position_construction.pose.orientation.w = cos(yaw_sp/2);
    }
}

// state_machine::FIXED_BOX_POSITION_P2M fix_box_position;
// void fixed_box_position_p2m_cb(const state_machine::FIXED_BOX_POSITION_P2M::ConstPtr& msg)
// {
//     fix_box_position = *msg;
//     if (fix_box_receive_enable == true)
//     {
//         fix_box_return.box1_x = fix_box_position.box1_x;
//         fix_box_return.box1_y = fix_box_position.box1_y;
//         fix_box_return.box1_z = fix_box_position.box1_z;

//         fix_box_return.box2_x = fix_box_position.box2_x;
//         fix_box_return.box2_y = fix_box_position.box2_y;
//         fix_box_return.box2_z = fix_box_position.box2_z;

//         fix_box_return.box3_x = fix_box_position.box3_x;
//         fix_box_return.box3_y = fix_box_position.box3_y;
//         fix_box_return.box3_z = fix_box_position.box3_z;

//         fix_box_return.box4_x = fix_box_position.box4_x;
//         fix_box_return.box4_y = fix_box_position.box4_y;
//         fix_box_return.box4_z = fix_box_position.box4_z;

//         fix_box_return.box5_x = fix_box_position.box5_x;
//         fix_box_return.box5_y = fix_box_position.box5_y;
//         fix_box_return.box5_z = fix_box_position.box5_z;

//         fix_box_return.box6_x = fix_box_position.box6_x;
//         fix_box_return.box6_y = fix_box_position.box6_y;
//         fix_box_return.box6_z = fix_box_position.box6_z;

//         fixed_box_pub.publish(fix_box_return);

//         ROS_INFO("fix_box_return.box1_x:%f",fix_box_return.box1_x);
//         ROS_INFO("fix_box_return.box1_y:%f",fix_box_return.box1_y);
//         ROS_INFO("fix_box_return.box1_z:%f",fix_box_return.box1_z);

//         ROS_INFO("fix_box_return.box2_x:%f",fix_box_return.box2_x);
//         ROS_INFO("fix_box_return.box2_y:%f",fix_box_return.box2_y);
//         ROS_INFO("fix_box_return.box2_z:%f",fix_box_return.box2_z);

//         ROS_INFO("fix_box_return.box3_x:%f",fix_box_return.box3_x);
//         ROS_INFO("fix_box_return.box3_y:%f",fix_box_return.box3_y);
//         ROS_INFO("fix_box_return.box3_z:%f",fix_box_return.box3_z);

//         ROS_INFO("fix_box_return.box4_x:%f",fix_box_return.box4_x);
//         ROS_INFO("fix_box_return.box4_y:%f",fix_box_return.box4_y);
//         ROS_INFO("fix_box_return.box4_z:%f",fix_box_return.box4_z);

//         ROS_INFO("fix_box_return.box5_x:%f",fix_box_return.box5_x);
//         ROS_INFO("fix_box_return.box5_y:%f",fix_box_return.box5_y);
//         ROS_INFO("fix_box_return.box5_z:%f",fix_box_return.box5_z);

//         ROS_INFO("fix_box_return.box6_x:%f",fix_box_return.box6_x);
//         ROS_INFO("fix_box_return.box6_y:%f",fix_box_return.box6_y);
//         ROS_INFO("fix_box_return.box6_z:%f",fix_box_return.box6_z);

//         position_box1.pose.position.x = fix_box_position.box1_y;
//         position_box1.pose.position.y = fix_box_position.box1_x;
//         position_box1.pose.position.z = OBSERVE_HEIGET - fix_box_position.box1_z;

//         position_box2.pose.position.x = fix_box_position.box2_y;
//         position_box2.pose.position.y = fix_box_position.box2_x;
//         position_box2.pose.position.z = OBSERVE_HEIGET - fix_box_position.box2_z;

//         position_box3.pose.position.x = fix_box_position.box3_y;
//         position_box3.pose.position.y = fix_box_position.box3_x;
//         position_box3.pose.position.z = OBSERVE_HEIGET - fix_box_position.box3_z;

//         position_box4.pose.position.x = fix_box_position.box4_y;
//         position_box4.pose.position.y = fix_box_position.box4_x;
//         position_box4.pose.position.z = OBSERVE_HEIGET - fix_box_position.box4_z;

//         position_box5.pose.position.x = fix_box_position.box5_y;
//         position_box5.pose.position.y = fix_box_position.box5_x;
//         position_box5.pose.position.z = OBSERVE_HEIGET - fix_box_position.box5_z;

//         position_box6.pose.position.x = fix_box_position.box6_y;
//         position_box6.pose.position.y = fix_box_position.box6_x;
//         position_box6.pose.position.z = OBSERVE_HEIGET - fix_box_position.box6_z;

//         fix_box_receive_enable = false;
//     }

// }

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
// }


/*****************************main function*****************************/
int main(int argc, char **argv)
{
    ros::init(argc, argv, "offboard_node");
    ros::NodeHandle nh;

    //takeoff velocity
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

    //descend to grab velocity
    vel_descend.twist.linear.x = 0.0f;
    vel_descend.twist.linear.y = 0.0f;
    vel_descend.twist.linear.z = -DESCEND_VELOCITY;
    vel_descend.twist.angular.x = 0.0f;
    vel_descend.twist.angular.y = 0.0f;
    vel_descend.twist.angular.z = 0.0f;

    //initial value
    // fix_target_return.home_x = 4;
    // fix_target_return.home_y = 4;
    // fix_target_return.home_z = 4;

    // fix_target_return.component_x = 4;
    // fix_target_return.component_y = 4;
    // fix_target_return.component_z = 4;

    // fix_target_return.construction_x = 4;
    // fix_target_return.construction_y = 4;
    // fix_target_return.construction_z = 4;

    // fix_target_return.box1_x = 4;
    // fix_target_return.box1_y = 4;
    // fix_target_return.box1_z = 4;

    // fix_target_return.box2_x = 4;
    // fix_target_return.box2_y = 4;
    // fix_target_return.box2_z = 4;

    // fix_target_return.box3_x = 4;
    // fix_target_return.box3_y = 4;
    // fix_target_return.box3_z = 4;

    // fix_target_return.box4_x = 4;
    // fix_target_return.box4_y = 4;
    // fix_target_return.box4_z = 4;

    // fix_target_return.box5_x = 4;
    // fix_target_return.box5_y = 4;
    // fix_target_return.box5_z = 4;

    //  float yaw_sp=M_PI_2;
    // //position_grab
    // position_grab.pose.orientation.x = 0;
    // position_grab.pose.orientation.y = 0;
    // position_grab.pose.orientation.z = sin(yaw_sp/2);
    // position_grab.pose.orientation.w = cos(yaw_sp/2);
    // //position_place
    // position_place.pose.orientation.x = 0;
    // position_place.pose.orientation.y = 0;
    // position_place.pose.orientation.z = sin(yaw_sp/2);
    // position_place.pose.orientation.w = cos(yaw_sp/2);
    // //position_judge
    // position_judge.pose.orientation.x = 0;
    // position_judge.pose.orientation.y = 0;
    // position_judge.pose.orientation.z = sin(yaw_sp/2);
    // position_judge.pose.orientation.w = cos(yaw_sp/2);
    // //position_of_safe
    // position_of_safe.pose.orientation.x = 0;
    // position_of_safe.pose.orientation.y = 0;
    // position_of_safe.pose.orientation.z = sin(yaw_sp/2);
    // position_of_safe.pose.orientation.w = cos(yaw_sp/2);
     
    //topic  subscribe
    ros::Subscriber state_sub = nh.subscribe<state_machine::State>("mavros/state",10,state_cb);
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose",10,pose_cb);
    //ros::Subscriber vel_sub = nh.subscribe<geometry_msgs::TwistStamped>("mavros/local_position/velocity",10,velo_cb);
    ros::Subscriber fixed_target_sub = nh.subscribe<state_machine::FIXED_TARGET_POSITION_P2M>("mavros/fixed_target_position_p2m",10,fixed_target_position_p2m_cb);
    ros::Subscriber task_status_sub = nh.subscribe<state_machine::TASK_STATUS_CHANGE_P2M>("mavros/task_status_change_p2m",10,task_status_change_p2m_cb);
    //ros::Subscriber fixed_box_sub = nh.subscribe<state_machine::FIXED_BOX_POSITION_P2M>("mavros/fixed_box_position_p2m",10,fixed_box_position_p2m_cb);

    //ros::Subscriber distance_sub = nh.subscribe<state_machine::Distance>("distance",10,distance_cb);

    //topic publish
    local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local",10);
    local_vel_pub = nh.advertise<geometry_msgs::TwistStamped>("mavros/setpoint_velocity/cmd_vel",10);
    fixed_target_pub = nh.advertise<state_machine::FIXED_TARGET_RETURN_M2P>("mavros/fixed_target_return_m2p",10);
    //fixed_box_pub = nh.advertise<state_machine::FIXED_BOX_RETURN_M2P>("mavros/fixed_box_return_m2p",10);
    ros::Publisher grab_status_pub = nh.advertise<state_machine::GRAB_STATUS_M2P>("mavros/grab_status_m2p",10);
    ros::Publisher task_status_pub = nh.advertise<state_machine::TASK_STATUS_MONITOR_M2P>("mavros/task_status_monitor_m2p",10);

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

        switch(box_loop)
        {
            case 0:
                position_grab.pose.position.x = position_box0.pose.position.x;
                position_grab.pose.position.y = position_box0.pose.position.y;
                position_grab.pose.position.z = -fix_target_position.component_z - grab_loop * BOX_HEIGET + GRAB_HEIGHT_MARGIN;;

                position_component.pose.position.x = position_box0.pose.position.x;
                position_component.pose.position.y = position_box0.pose.position.y;
                position_component.pose.position.z = position_box0.pose.position.z;

            break;
            case 1:
                position_grab.pose.position.x = position_box1.pose.position.x;
                position_grab.pose.position.y = position_box1.pose.position.y;
                position_grab.pose.position.z = -fix_target_position.box1_z - grab_loop * BOX_HEIGET + GRAB_HEIGHT_MARGIN;

                position_component.pose.position.x = position_box1.pose.position.x;
                position_component.pose.position.y = position_box1.pose.position.y;
                position_component.pose.position.z = position_box1.pose.position.z;

            break;
            case 2:
                position_grab.pose.position.x = position_box2.pose.position.x;
                position_grab.pose.position.y = position_box2.pose.position.y;
                position_grab.pose.position.z = -fix_target_position.box2_z - grab_loop * BOX_HEIGET + GRAB_HEIGHT_MARGIN;

                position_component.pose.position.x = position_box2.pose.position.x;
                position_component.pose.position.y = position_box2.pose.position.y;
                position_component.pose.position.z = position_box2.pose.position.z;

            break;
            case 3:
                position_grab.pose.position.x = position_box3.pose.position.x;
                position_grab.pose.position.y = position_box3.pose.position.y;
                position_grab.pose.position.z = -fix_target_position.box3_z - grab_loop * BOX_HEIGET + GRAB_HEIGHT_MARGIN;

                position_component.pose.position.x = position_box3.pose.position.x;
                position_component.pose.position.y = position_box3.pose.position.y;
                position_component.pose.position.z = position_box3.pose.position.z;

            break;
            case 4:
                position_grab.pose.position.x = position_box4.pose.position.x;
                position_grab.pose.position.y = position_box4.pose.position.y;
                position_grab.pose.position.z = -fix_target_position.box4_z - grab_loop * BOX_HEIGET + GRAB_HEIGHT_MARGIN;

                position_component.pose.position.x = position_box4.pose.position.x;
                position_component.pose.position.y = position_box4.pose.position.y;
                position_component.pose.position.z = position_box4.pose.position.z;

            break;
            case 5:
                position_grab.pose.position.x = position_box5.pose.position.x;
                position_grab.pose.position.y = position_box5.pose.position.y;
                position_grab.pose.position.z = -fix_target_position.box5_z - grab_loop * BOX_HEIGET + GRAB_HEIGHT_MARGIN;

                position_component.pose.position.x = position_box5.pose.position.x;
                position_component.pose.position.y = position_box5.pose.position.y;
                position_component.pose.position.z = position_box5.pose.position.z;

            break;
        }

        task_status_monitor.task_status = current_pos_state;
        task_status_monitor.loop_value = place_loop;
        task_status_monitor.target_x = pose_pub.pose.position.y;
        task_status_monitor.target_y = pose_pub.pose.position.x;
        task_status_monitor.target_z = -pose_pub.pose.position.z;
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
            if((current_position.pose.position.z + fix_target_position.home_z) > 3)
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
            pose_pub = position_component;
            local_pos_pub.publish(position_component);
            if (Distance_of_Two(current_position.pose.position.x,position_component.pose.position.x,
                                current_position.pose.position.y,position_component.pose.position.y,
                                current_position.pose.position.z,position_component.pose.position.z) < LOCATE_ACCURACY_HIGH)
            {
                // position_grab.pose.position.x = position_component.pose.position.x;
                // position_grab.pose.position.y = position_component.pose.position.y;
                // position_grab.pose.position.z = -fix_target_position.component_z - loop * BOX_HEIGET + GRAB_HEIGHT_MARGIN;

                current_pos_state = position_Com_hover;
                last_time = ros::Time::now();
            }
        }
        break;
        case position_Com_hover:
        {
            static int accuracy_count1 = 0;  //for improve accuracy
            static int hover_count1 = 0;
            pose_pub = position_component;
            local_pos_pub.publish(position_component);
            if(ros::Time::now() - last_time > ros::Duration(2.0) && hover_count1 == 0)
            {
                hover_count1++;
            }
            if (ros::Time::now() - last_time > ros::Duration(0.5) && hover_count1 > 0)
            {
                if (Distance_of_Two(current_position.pose.position.x,position_component.pose.position.x,
                                    current_position.pose.position.y,position_component.pose.position.y,
                                    current_position.pose.position.z,position_component.pose.position.z) < 0.15)
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
                if(grab_count > 40)
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
            pose_pub = position_component;
            local_vel_pub.publish(vel_ascend_com);
            if ((current_position.pose.position.z + fix_target_position.component_z) > 2)
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
            // static int grab_judge_count = 0;
            // pose_pub = position_judge;
            // local_pos_pub.publish(position_judge);
            // if(ros::Time::now() - last_time > ros::Duration(1.0) && grab_judge_count == 0)
            // {
            //     grab_judge_count++;
            // }
            // if (ros::Time::now() - last_time > ros::Duration(0.5) && grab_judge_count >0)
            // {
            //     if (distance.distance < 0.5)
            //         {
            //             grab_judge_count++;
            //             if(grab_judge_count > 10)
            //             {
            //                 grab_judge_count = 0;
            //                 current_pos_state = position_Con_go;
            //                 last_time = ros::Time::now();
            //                 break;
            //             }
            //         }
            //     else
            //     {
            //         grab_judge_count = 0;
            //         current_pos_state = position_Com_go;
            //         last_time = ros::Time::now();
            //         break;
            //     }               
            // }
            pose_pub = position_judge;
            local_pos_pub.publish(position_judge);
            if(ros::Time::now() - last_time > ros::Duration(3.0))
            {
                box_loop++;
                if(box_loop == BOX_LOOP_MAX)
                {
                    box_loop = 0;
                    grab_loop++;
                }
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
                position_place.pose.position.z = -fix_target_position.construction_z + place_loop * BOX_HEIGET + PLACE_HEIGET;

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
                                    current_position.pose.position.z,position_place.pose.position.z) < 0.1)
                {
                    accuracy_count3++;
                    if(accuracy_count3 > 3)
                    {
                        current_pos_state = componnet_place;
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
                current_pos_state = componnet_place;
                accuracy_count3 = 0;
                hover_count3 = 0;
                last_time = ros::Time::now();
                break;
            }
        }
        break;
        case componnet_place:
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
            if ((current_position.pose.position.z + fix_target_position.construction_z) > (3 + BOX_HEIGET*place_loop))
            {
                position_of_safe.pose.position.x = current_position.pose.position.x;
                position_of_safe.pose.position.y = current_position.pose.position.y;
                position_of_safe.pose.position.z = current_position.pose.position.z;
                current_pos_state = place_status_judge;
                last_time = ros::Time::now();
            }
        }
        break;
        case place_status_judge:
        {
            // static int place_judge_count = 0;
            // pose_pub = position_of_safe;
            // local_pos_pub.publish(position_of_safe);
            // if(ros::Time::now() - last_time > ros::Duration(1.0) && place_judge_count == 0)
            // {
            //     place_judge_count++;
            // }
            // if (ros::Time::now() - last_time > ros::Duration(0.5) && place_judge_count >0)
            // {
            //     if (distance.distance > 1.0)
            //         {
            //             place_judge_count++;
            //             if(place_judge_count > 3)
            //             {
            //                 place_judge_count = 0;
            //                 current_pos_state = return_home;
            //                 loop++;
            //                 last_time = ros::Time::now();
            //                 break;
            //             }
            //         }
            //     else
            //     {
            //         place_judge_count = 0;
            //         current_pos_state = place_point_get_close;
            //         last_time = ros::Time::now();
            //         break;
            //     }               
            // }
            pose_pub = position_of_safe;
            local_pos_pub.publish(position_of_safe);
            if(ros::Time::now() - last_time > ros::Duration(3.0))
            {
                place_loop++;
                if(place_loop < BOX_LOOP_MAX * 2)
                    current_pos_state = position_Com_go;
                else
                    current_pos_state = return_home;
                last_time = ros::Time::now();
            }
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

// limit angle_rad to [-pi,pi]
float wrap_pi(float angle_rad)
{
    if (angle_rad >= M_PI) 
    {
        angle_rad -= M_PI*2;
    }
    if (angle_rad < -M_PI) 
    {
        angle_rad += M_PI*2;
    }
    return angle_rad;
}