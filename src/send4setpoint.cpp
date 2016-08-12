/*
 * Func: sent 4 setpoints(A,B,C,D)
 *
 * -libn Aug 10, 2016 2:56:53 PM
 */
#include "ros/ros.h"
#include "state_machine/Setpoint.h"
#include "state_machine/State.h"
#include <geometry_msgs/PoseStamped.h>  /* message type of /mavros/local_position/pose (P.S. It is included in dir: /opt/ros/indigo/share/geometry_msgs/msg) -libn */
#include "state_machine/ActuatorControl.h" /* add actuator_control output */
#include <stdio.h>
#include <math.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "send4setpoints");

    ros::NodeHandle nh;

	/* send 1 setpoint positon -libn <Aug 10, 2016 2:16:47 PM> */
	state_machine::Setpoint setpoint;
	ros::Publisher setpoint_A_sub = nh.advertise<state_machine::Setpoint>("Setpoint_A", 10);
	ros::Publisher setpoint_B_sub = nh.advertise<state_machine::Setpoint>("Setpoint_B", 10);
	ros::Publisher setpoint_C_sub = nh.advertise<state_machine::Setpoint>("Setpoint_C", 10);
	ros::Publisher setpoint_D_sub = nh.advertise<state_machine::Setpoint>("Setpoint_D", 10);
	setpoint.x = 0.1f;
	setpoint.y = 0.2f;
	setpoint.z = sin(30*M_PI/180);

	ros::Rate loop_rate(10);

	int i;

	while(ros::ok())
	{
		for(i=1;i<180;i++)
		{
			// send 4 setpoints -libn
			setpoint.x = sin(i*M_PI/180);
			setpoint.y = sin((i+30)*M_PI/180);
			setpoint.z = sin((i+60)*M_PI/180);
			setpoint_A_sub.publish(setpoint);
			setpoint.x = sin(i*M_PI/180)+1;
			setpoint.y = sin((i+30)*M_PI/180)+1;
			setpoint.z = sin((i+60)*M_PI/180)+1;
			setpoint_B_sub.publish(setpoint);
			setpoint.x = sin(i*M_PI/180)+2;
			setpoint.y = sin((i+30)*M_PI/180)+2;
			setpoint.z = sin((i+60)*M_PI/180)+2;
			setpoint_C_sub.publish(setpoint);
			setpoint.x = sin(i*M_PI/180)+3;
			setpoint.y = sin((i+30)*M_PI/180)+3;
			setpoint.z = sin((i+60)*M_PI/180)+3;
			setpoint_D_sub.publish(setpoint);

			ros::spinOnce();

			loop_rate.sleep();

		}

	}

	return 0;
		
}
