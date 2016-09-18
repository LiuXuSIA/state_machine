/**
* @file     : send10picture_position.cpp
* @brief    : get 10 numbers and 10 positions([x,y,z]).
* @author   : libn
* @time     : Aug 15, 2016 10:38:53 AM
*/
#include "ros/ros.h"
#include <state_machine/State.h>
#include <geometry_msgs/PoseStamped.h>  /* message type of /mavros/local_position/pose (P.S. It is included in dir: /opt/ros/indigo/share/geometry_msgs/msg) -libn */
#include <state_machine/ActuatorControl.h> /* add actuator_control output */
#include <stdio.h>
#include <math.h>
#include <state_machine/DrawingBoard.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "send10picture_position");

    ros::NodeHandle nh;

    /* send indexed setpoint. -libn <Aug 15, 2016 9:00:02 AM> */
    state_machine::DrawingBoard board;
    ros::Publisher DrawingBoard_Position_pub = nh.advertise<state_machine::DrawingBoard>("DrawingBoard_Position", 10);

    board.num = 1;
    board.x = 0.5f;
    board.y = 0.6f;
    board.z = 0.7f;

	ros::Rate loop_rate(10);

	while(ros::ok())
	{

		board.num = 0;
        board.x = 2.84f;
        board.y = 2.99f;
        board.z = 2.5f;
		board.valid = true;
		DrawingBoard_Position_pub.publish(board);
		ros::spinOnce();
		loop_rate.sleep();

		board.num = 1;
		board.x = 0.1f;
		board.y = 0.1f;
		board.z = 0.1f;
		board.valid = true;
		DrawingBoard_Position_pub.publish(board);
		ros::spinOnce();
		loop_rate.sleep();

		board.num = 2;
		board.x = 0.2f;
		board.y = 0.2f;
		board.z = 0.2f;
		board.valid = true;
		DrawingBoard_Position_pub.publish(board);
		ros::spinOnce();
		loop_rate.sleep();

		board.num = 3;
		board.x = 0.3f;
		board.y = 0.3f;
		board.z = 0.3f;
		board.valid = true;
		DrawingBoard_Position_pub.publish(board);
		ros::spinOnce();
		loop_rate.sleep();

		board.num = 4;
		board.x = 0.4f;
		board.y = 0.4f;
		board.z = 0.4f;
		board.valid = true;
		DrawingBoard_Position_pub.publish(board);
		ros::spinOnce();
		loop_rate.sleep();

		board.num = 5;
		board.x = 0.5f;
		board.y = 0.5f;
		board.z = 0.5f;
		board.valid = true;
		DrawingBoard_Position_pub.publish(board);
		ros::spinOnce();
		loop_rate.sleep();

		board.num = 6;
		board.x = 0.6f;
		board.y = 0.6f;
		board.z = 0.6f;
		board.valid = true;
		DrawingBoard_Position_pub.publish(board);
		ros::spinOnce();
		loop_rate.sleep();

		board.num = 7;
		board.x = 0.7f;
		board.y = 0.7f;
		board.z = 0.7f;
		board.valid = true;
		DrawingBoard_Position_pub.publish(board);
		ros::spinOnce();
		loop_rate.sleep();

		board.num = 8;
		board.x = 0.8f;
		board.y = 0.8f;
		board.z = 0.8f;
		board.valid = true;
		DrawingBoard_Position_pub.publish(board);
		ros::spinOnce();
		loop_rate.sleep();

		board.num = 9;
		board.x = 0.9f;
		board.y = 0.9f;
		board.z = 0.9f;
		board.valid = true;
		DrawingBoard_Position_pub.publish(board);
		ros::spinOnce();
		loop_rate.sleep();


	}

	return 0;
		
}
