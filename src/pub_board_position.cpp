/**
* @file     : pub_board_position.cpp (just for test)
* @brief    : publish 10 boards' positions.
* @author   : libn
* @time     : Sep 23, 2016 3:46:32 PM
*/

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <state_machine/DrawingBoard.h>
#include <state_machine/DrawingBoard10.h>
#include <geometry_msgs/PoseStamped.h>

ros::Publisher DrawingBoard_Position_pub;

ros::Time last_request;

//// local position msg callback function
//geometry_msgs::PoseStamped current_pos;
//void pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
//{
//    current_pos = *msg;
//}

/* send indexed setpoint. -libn <Aug 15, 2016 9:00:02 AM> */
state_machine::DrawingBoard board;
state_machine::DrawingBoard10 board10;

sensor_msgs::LaserScan board_scan;

int main(int argc, char **argv)
{
	ROS_INFO("I was alive.");
	ros::init(argc, argv, "pub_board_pos");
	ros::NodeHandle nh;

    DrawingBoard_Position_pub = nh.advertise<state_machine::DrawingBoard10>("DrawingBoard_Position10", 1);

//	ros::Subscriber board_pos_sub = nh.subscribe<sensor_msgs::LaserScan>
//	            ("/vision/digit_nws_position", 10, board_pos_cb);
	board10.drawingboard.resize(10);		/* MUST! -libn */

//	/* get pixhawk's local position. -libn */
//	ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, pos_cb);

	ros::Rate loop_rate(10);

	for(int i = 0; i < 10; i++)
	{
        board10.drawingboard[i].num = i;
        board10.drawingboard[i].x = -(float)i;
        board10.drawingboard[i].y = (float)i;
        board10.drawingboard[i].z = 0.0f;
        board10.drawingboard[i].valid = true;

	}

	while(ros::ok())
	{
		DrawingBoard_Position_pub.publish(board10);
		ROS_INFO("Publishing!");
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
