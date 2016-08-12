/**
* @file     : send4setpoint.cpp
* @brief    : sent 4 setpoints(A,B,C,D)
* @author   : libn
* @time     : Aug 12, 2016 3:57:08 PM
*/
#include "ros/ros.h"
#include "state_machine/State.h"
#include <geometry_msgs/PoseStamped.h>  /* message type of /mavros/local_position/pose (P.S. It is included in dir: /opt/ros/indigo/share/geometry_msgs/msg) -libn */
#include "state_machine/ActuatorControl.h" /* add actuator_control output */
#include <stdio.h>
#include <math.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "send4setpoints");

    ros::NodeHandle nh;

	/* send 4 setpoint positon -libn <Aug 10, 2016 2:16:47 PM> */
    geometry_msgs::PoseStamped setpoint;
	ros::Publisher setpoint_A_pub = nh.advertise<geometry_msgs::PoseStamped>("Setpoint_A", 10);
	ros::Publisher setpoint_B_pub = nh.advertise<geometry_msgs::PoseStamped>("Setpoint_B", 10);
	ros::Publisher setpoint_C_pub = nh.advertise<geometry_msgs::PoseStamped>("Setpoint_C", 10);
	ros::Publisher setpoint_D_pub = nh.advertise<geometry_msgs::PoseStamped>("Setpoint_D", 10);

	setpoint.pose.position.x = 0.1f;
	setpoint.pose.position.y = 0.2f;
	setpoint.pose.position.z = 0.3f;


	ros::Rate loop_rate(10);

	int i;

	while(ros::ok())
	{
		for(i=1;i<180;i++)
		{
			// send 4 setpoints -libn
			setpoint.pose.position.x = sin(i*M_PI/180);
			setpoint.pose.position.y = sin((i+30)*M_PI/180);
			setpoint.pose.position.z = sin((i+60)*M_PI/180);
			setpoint_A_pub.publish(setpoint);
			setpoint.pose.position.x = sin(i*M_PI/180)+1;
			setpoint.pose.position.y = sin((i+30)*M_PI/180)+1;
			setpoint.pose.position.z = sin((i+60)*M_PI/180)+1;
			setpoint_B_pub.publish(setpoint);
			setpoint.pose.position.x = sin(i*M_PI/180)+2;
			setpoint.pose.position.y = sin((i+30)*M_PI/180)+2;
			setpoint.pose.position.z = sin((i+60)*M_PI/180)+2;
			setpoint_C_pub.publish(setpoint);
			setpoint.pose.position.x = sin(i*M_PI/180)+3;
			setpoint.pose.position.y = sin((i+30)*M_PI/180)+3;
			setpoint.pose.position.z = sin((i+60)*M_PI/180)+3;
			setpoint_D_pub.publish(setpoint);

			ros::spinOnce();

			loop_rate.sleep();

		}

	}

	return 0;
		
}
