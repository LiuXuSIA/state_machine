/*************************************************************************
@file           coordinate_exchange.cpp
@date           2018/10/12 
@author         liuxu
@email          liuxu.ccc@gmail.com
@description    coordinate_exchange,get the bias of camera coordinate and aerial body coordinate

                first,put aerial in one place,get its local NED coorninate by GCS,(x1,y1,z1)
                then,move the aerial to another place,get its local NED coorninate(x2,y2,z2) 
                and its accurate camera coordinate relative to the former position(x3,y3,z3)
                more,calculate the airborne NED coordinate by (x2,y2,z2) - (x1,y1,z1)
                further,calculate the body coordinate of the latter position relative to the former point
                by R*((x2,y2,z2) - (x1,y1,z1)),where R is the exchge matrix
                and we can get the bias by compare (x3,y3,z3) and R*((x2,y2,z2) - (x1,y1,z1))
                
*************************************************************************/


/****************************header files********************************/
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include <state_machine/FIXED_TARGET_POSITION_P2M.h>
#include <state_machine/FIXED_TARGET_RETURN_M2P.h>

#include <state_machine/Attitude.h>
#include "math.h"


/*************************constant defunition***************************/

#define HOME_HEIGHT            2.5
#define ASCEND_VELOCITY        1.5
#define OBSERVE_HEIGET         2.5
#define CONSTRUCTION_HEIGET    2.5
#define LOCATE_ACCURACY        0.6

/***************************variable definition*************************/
//position  ENU
geometry_msgs::PoseStamped position_home;
geometry_msgs::PoseStamped position_componnet;
geometry_msgs::PoseStamped position_construction;
geometry_msgs::PoseStamped body_ned;
geometry_msgs::PoseStamped body;

float bias_x;
float bias_y;
float bias_z;

ros::Publisher fixed_target_pub;

float R[3][3] = {0};

//message to pix
state_machine::FIXED_TARGET_RETURN_M2P fix_target_return;

geometry_msgs::PoseStamped current_position;
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    current_position = *msg;
}

state_machine::Attitude current_attitude;
void attitude_cb(const state_machine::Attitude::ConstPtr& msg)
{
    current_attitude = *msg;

    R[0][0] = cos(current_attitude.pitch) * cos(current_attitude.yaw);
    R[0][1] = cos(current_attitude.pitch) * sin(current_attitude.yaw);
    R[0][2] = -sin(current_attitude.pitch);

    R[1][0] = sin(current_attitude.roll) * sin(current_attitude.pitch) * cos(current_attitude.yaw) -
              cos(current_attitude.roll) * sin(current_attitude.yaw);
    R[1][1] = sin(current_attitude.roll) * sin(current_attitude.pitch) * sin(current_attitude.yaw) +
              cos(current_attitude.roll) * cos(current_attitude.yaw);
    R[1][2] = cos(current_attitude.pitch) * sin(current_attitude.yaw);

    R[2][0] = cos(current_attitude.roll) * sin(current_attitude.pitch) * cos(current_attitude.yaw) +
              sin(current_attitude.roll) * sin(current_attitude.yaw);
    R[2][1] = cos(current_attitude.roll) * sin(current_attitude.pitch) * sin(current_attitude.yaw) -
              sin(current_attitude.roll) * cos(current_attitude.yaw);
    R[2][2]= cos(current_attitude.roll) * cos(current_attitude.pitch);
}

state_machine::FIXED_TARGET_POSITION_P2M fix_target_position;
void fixed_target_position_p2m_cb(const state_machine::FIXED_TARGET_POSITION_P2M::ConstPtr& msg)
{
    fix_target_position = *msg;
    
    fix_target_return.home_x = fix_target_position.home_x;
    fix_target_return.home_y = fix_target_position.home_y;
    fix_target_return.home_z = -HOME_HEIGHT + fix_target_position.home_z;

    fix_target_return.component_x = fix_target_position.component_x;
    fix_target_return.component_y = fix_target_position.component_y;
    fix_target_return.component_z = -OBSERVE_HEIGET + fix_target_position.component_z;

    fix_target_return.construction_x = fix_target_position.construction_x;
    fix_target_return.construction_y = fix_target_position.construction_y;
    fix_target_return.construction_z = -CONSTRUCTION_HEIGET + fix_target_position.construction_z;

    body_ned.pose.position.x = fix_target_position.component_x - fix_target_position.home_x;
    body_ned.pose.position.y = fix_target_position.component_y - fix_target_position.home_y;
    body_ned.pose.position.z = fix_target_position.component_z - fix_target_position.home_z;

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

    body.pose.position.x = (R[0][0] * body_ned.pose.position.x + R[0][1] * body_ned.pose.position.y + R[0][2] * body_ned.pose.position.z);
    body.pose.position.y = (R[1][0] * body_ned.pose.position.x + R[1][1] * body_ned.pose.position.y + R[1][2] * body_ned.pose.position.z);
    body.pose.position.z = (R[2][0] * body_ned.pose.position.x + R[2][1] * body_ned.pose.position.y + R[2][2] * body_ned.pose.position.z);

    bias_x = body.pose.position.x - 1;
    bias_y = body.pose.position.x - 1;
    bias_z = body.pose.position.x - 1;
}

/*****************************main function*****************************/
int main(int argc, char **argv)
{
    ros::init(argc, argv, "offboard_node");
    ros::NodeHandle nh;
     
    //topic  subscribe
    ros::Subscriber fixed_target_sub = nh.subscribe<state_machine::FIXED_TARGET_POSITION_P2M>("mavros/fixed_target_position_p2m",10,fixed_target_position_p2m_cb);
    ros::Subscriber pose_sub = nh.subscribe<state_machine::Attitude>("mavros/attitude",10,attitude_cb);

    
    //topic publish
    fixed_target_pub = nh.advertise<state_machine::FIXED_TARGET_RETURN_M2P>("mavros/fixed_target_return_m2p",10);

    ros::Rate rate(10.0);

    while(ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }
    
    return 0;
}