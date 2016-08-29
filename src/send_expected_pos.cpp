/**
* @file     : send_expected_pos.cpp(for test)
* @brief    : 1)get 4 setpoints; 2)send expected position to pix
* @author   : libn
* @time     : Aug 12, 2016 2:16:06 PM
*/
#include "ros/ros.h"
#include <state_machine/Attitude.h>
#include <geometry_msgs/PoseStamped.h>  /* message type of /mavros/local_position/pose (P.S. It is included in dir: /opt/ros/indigo/share/geometry_msgs/msg) -libn */
#include <stdio.h>

// position setpoint to publish
geometry_msgs::PoseStamped setpoint_pub;

// pulisher: used to publish local_pos_setpoint -libn
ros::Publisher local_pos_setpoint_pub;

/* get 4 setpoints. -libn <Aug 12, 2016 2:19:27 PM> */
geometry_msgs::PoseStamped setpoint_A;
geometry_msgs::PoseStamped setpoint_B;
geometry_msgs::PoseStamped setpoint_C;
geometry_msgs::PoseStamped setpoint_D;
void printSetpointACallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
//    ROS_INFO("I heard: [%f] [%f] [%f]",msg->x, msg->y, msg->z);
    setpoint_A = *msg;
}
void printSetpointBCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
//    ROS_INFO("I heard: [%f] [%f] [%f]",msg->x, msg->y, msg->z);
    setpoint_B = *msg;
}
void printSetpointCCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
//    ROS_INFO("I heard: [%f] [%f] [%f]",msg->x, msg->y, msg->z);
    setpoint_C = *msg;
}
void printSetpointDCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
//    ROS_INFO("I heard: [%f] [%f] [%f]",msg->x, msg->y, msg->z);
    setpoint_D = *msg;
}


void printATTCallback(const state_machine::Attitude::ConstPtr& msg)
{
	ROS_INFO("I heard:[%f] [%f] [%f]",msg->roll, msg->pitch, msg->yaw);
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "send_expected_pos");

	ros::NodeHandle nh;

	ros::Subscriber ATT_sub = nh.subscribe("mavros/attitude", 10 ,printATTCallback);

	/* receive 4 setpoints. -libn <Aug 10, 2016 2:57:25 PM> */
	ros::Subscriber setpoint_A_sub = nh.subscribe("Setpoint_A", 100 ,printSetpointACallback);
	ros::Subscriber setpoint_B_sub = nh.subscribe("Setpoint_B", 100 ,printSetpointBCallback);
	ros::Subscriber setpoint_C_sub = nh.subscribe("Setpoint_C", 100 ,printSetpointCCallback);
	ros::Subscriber setpoint_D_sub = nh.subscribe("Setpoint_D", 100 ,printSetpointDCallback);
	ROS_INFO("setpoints suscribe start!");

	setpoint_pub = setpoint_A;

	ros::Rate rate(10);		/* 10Hz. -libn <Aug 11, 2016 9:28:08 AM> */

	/* publish local_pos_setpoint -libn <Aug 11, 2016 10:05:05 AM> */
	ros::Publisher local_pos_setpoint_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
	//send a few setpoints before starting  --> for safety
	for(int i = 100; ros::ok() && i > 0; --i){
		setpoint_pub = setpoint_A;			// set expected position
		local_pos_setpoint_pub.publish(setpoint_pub);      // publish pos A (desired position)
		ros::spinOnce();
		rate.sleep();
	}

	while(ros::ok())
	{
		ROS_INFO("setpoint_A received: [%f] [%f] [%f]",setpoint_A.pose.position.x, setpoint_A.pose.position.y, setpoint_A.pose.position.z);
		ROS_INFO("setpoint_B received: [%f] [%f] [%f]",setpoint_B.pose.position.x, setpoint_B.pose.position.y, setpoint_B.pose.position.z);
		ROS_INFO("setpoint_C received: [%f] [%f] [%f]",setpoint_C.pose.position.x, setpoint_C.pose.position.y, setpoint_C.pose.position.z);
		ROS_INFO("setpoint_D received: [%f] [%f] [%f]",setpoint_D.pose.position.x, setpoint_D.pose.position.y, setpoint_D.pose.position.z);
		setpoint_pub = setpoint_A;			// set expected position
		local_pos_setpoint_pub.publish(setpoint_pub);      // publish pos A (desired position)


		ros::spinOnce();	/* refresh data subscriber. -libn <Aug 11, 2016 9:28:33 AM> */
		rate.sleep();

	}


	return 0;
		
}

