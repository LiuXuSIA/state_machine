/*******************************************************************************
@file         send3point.cpp
@date         20180821  08:35
@author       liuxu
@email        lixu.ccc@gmail.com
@description  send 3 position just for simulation
********************************************************************************/
#include "ros/ros.h"
#include "state_machine/State.h"
#include "geometry_msgs/PoseStamped.h"
#include "state_machine/Setpoint.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "send3point");

	ros::NodeHandle nh;

	state_machine::Setpoint setpoint;
	ros::Publisher setpoint_pub = nh.advertise<state_machine::Setpoint>("set3point",10);

	ros::Rate loop_rate(10);

	while(ros::ok())
	{
		setpoint.index = 1;
		setpoint.x = 0;                        
		setpoint.y = 0;
		setpoint.z = 3;
		setpoint_pub.publish(setpoint);
		ros::spinOnce();
		loop_rate.sleep();

		setpoint.index = 2;
		setpoint.x = 0;
		setpoint.y = 0;
		setpoint.z = 1;
		setpoint_pub.publish(setpoint);
		ros::spinOnce();
		loop_rate.sleep();

		setpoint.index = 3;
		setpoint.x = 0;
		setpoint.y = 0;
		setpoint.z = 2;
		setpoint_pub.publish(setpoint);
		ros::spinOnce();
		loop_rate.sleep();	
	}

	return 0;
}