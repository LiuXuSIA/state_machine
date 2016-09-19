/**
* @file     : get_board_position_receive.cpp
* @brief    : subscribe the position of 10 boards(just for test).
* @author   : libn
* @time     : Sep 19, 2016 1:27:52 PM
*/

#include <ros/ros.h>
#include <state_machine/DrawingBoard10.h>

/* send indexed setpoint. -libn <Aug 15, 2016 9:00:02 AM> */
state_machine::DrawingBoard10 board10;

void board_pos_cb(const state_machine::DrawingBoard10::ConstPtr& msg)
{
	board10 = *msg;

	ROS_INFO("\nboard_0 position: %d x = %f y = %f z = %f\n"
				"board_1 position: %d x = %f y = %f z = %f\n"
				"board_2 position: %d x = %f y = %f z = %f\n"
				"board_3 position: %d x = %f y = %f z = %f\n"
				"board_4 position: %d x = %f y = %f z = %f\n"
				"board_5 position: %d x = %f y = %f z = %f\n"
				"board_6 position: %d x = %f y = %f z = %f\n"
				"board_7 position: %d x = %f y = %f z = %f\n"
				"board_8 position: %d x = %f y = %f z = %f\n"
				"board_9 position: %d x = %f y = %f z = %f\n",
				board10.drawingboard[0].valid,board10.drawingboard[0].x,board10.drawingboard[0].y,board10.drawingboard[0].z,
				board10.drawingboard[1].valid,board10.drawingboard[1].x,board10.drawingboard[1].y,board10.drawingboard[1].z,
				board10.drawingboard[2].valid,board10.drawingboard[2].x,board10.drawingboard[2].y,board10.drawingboard[2].z,
				board10.drawingboard[3].valid,board10.drawingboard[3].x,board10.drawingboard[3].y,board10.drawingboard[3].z,
				board10.drawingboard[4].valid,board10.drawingboard[4].x,board10.drawingboard[4].y,board10.drawingboard[4].z,
				board10.drawingboard[5].valid,board10.drawingboard[5].x,board10.drawingboard[5].y,board10.drawingboard[5].z,
				board10.drawingboard[6].valid,board10.drawingboard[6].x,board10.drawingboard[6].y,board10.drawingboard[6].z,
				board10.drawingboard[7].valid,board10.drawingboard[7].x,board10.drawingboard[7].y,board10.drawingboard[7].z,
				board10.drawingboard[8].valid,board10.drawingboard[8].x,board10.drawingboard[8].y,board10.drawingboard[8].z,
				board10.drawingboard[9].valid,board10.drawingboard[9].x,board10.drawingboard[9].y,board10.drawingboard[9].z);
}


int main(int argc, char **argv)
{
	ROS_INFO("I was alive.");
	ros::init(argc, argv, "receive_board_pos");
	ros::NodeHandle nh;

//    DrawingBoard_Position_pub = nh.advertise<state_machine::DrawingBoard10>("DrawingBoard_Position10", 1);

	ros::Subscriber DrawingBoard_Position_sub = nh.subscribe<state_machine::DrawingBoard10>
	            ("DrawingBoard_Position10", 10, board_pos_cb);
	board10.drawingboard.resize(10);		/* MUST! -libn */

	ros::spin();
	return 0;
}
