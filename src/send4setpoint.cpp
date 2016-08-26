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
#include "state_machine/Setpoint.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "send4setpoints");

    ros::NodeHandle nh;

    /* send indexed setpoint. -libn <Aug 15, 2016 9:00:02 AM> */
    state_machine::Setpoint setpoint_indexed;
    ros::Publisher setpoint_indexed_pub = nh.advertise<state_machine::Setpoint>("Setpoint_Indexed", 10);

    setpoint_indexed.index = 1;	/* 1st setpoint. -libn <Aug 15, 2016 9:01:21 AM> */
    setpoint_indexed.x = 2.1f;
    setpoint_indexed.y = 2.1f;
    setpoint_indexed.z = 2.1f;

	ros::Rate loop_rate(10);

	while(ros::ok())
	{
		setpoint_indexed.index = 1;	/* 1st setpoint. -libn <Aug 15, 2016 9:01:21 AM> */
		setpoint_indexed.x = 4.1f;
		setpoint_indexed.y = 4.1f;
		setpoint_indexed.z = 2.1f;
		setpoint_indexed_pub.publish(setpoint_indexed);
		ros::spinOnce();
		loop_rate.sleep();

		setpoint_indexed.index = 2;	/* 2ed setpoint. -libn <Aug 15, 2016 9:01:21 AM> */
		setpoint_indexed.x = 6.2f;
		setpoint_indexed.y = 6.2f;
		setpoint_indexed.z = 2.1f;
		setpoint_indexed_pub.publish(setpoint_indexed);
		ros::spinOnce();
		loop_rate.sleep();

		setpoint_indexed.index = 3;	/* 3rd setpoint. -libn <Aug 15, 2016 9:01:21 AM> */
		setpoint_indexed.x = 8.3f;
		setpoint_indexed.y = 8.3f;
		setpoint_indexed.z = 2.1f;
		setpoint_indexed_pub.publish(setpoint_indexed);
		ros::spinOnce();
		loop_rate.sleep();

		setpoint_indexed.index = 4;	/* 4th setpoint. -libn <Aug 15, 2016 9:01:21 AM> */
		setpoint_indexed.x = 10.4f;
		setpoint_indexed.y = 10.4f;
		setpoint_indexed.z = 2.1f;
		setpoint_indexed_pub.publish(setpoint_indexed);
		ros::spinOnce();
		loop_rate.sleep();


	}

	return 0;
		
}
