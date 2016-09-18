/**
* @file     : get_board_position.cpp
* @brief    : subscribe the position of 10 boards(just for test).
* @author   : libn
* @time     : Sep 17, 2016 9:50:58 PM
*/

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

sensor_msgs::LaserScan board_position;
void board_pos_cb(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	board_position = *msg;
	ROS_INFO("board_0 position: x = %f y = %f z = %f\n"
			"board_1 position: x = %f y = %f z = %f\n"
			"board_2 position: x = %f y = %f z = %f\n"
			"board_3 position: x = %f y = %f z = %f\n"
			"board_4 position: x = %f y = %f z = %f\n"
			"board_5 position: x = %f y = %f z = %f\n"
			"board_6 position: x = %f y = %f z = %f\n"
			"board_7 position: x = %f y = %f z = %f\n"
			"board_8 position: x = %f y = %f z = %f\n"
			"board_9 position: x = %f y = %f z = %f\n",
			board_position.ranges[0],board_position.ranges[1],board_position.ranges[2],
			board_position.ranges[3],board_position.ranges[4],board_position.ranges[5],
			board_position.ranges[6],board_position.ranges[7],board_position.ranges[8],
			board_position.ranges[9],board_position.ranges[10],board_position.ranges[11],
			board_position.ranges[12],board_position.ranges[13],board_position.ranges[14],
			board_position.ranges[15],board_position.ranges[16],board_position.ranges[17],
			board_position.ranges[18],board_position.ranges[19],board_position.ranges[20],
			board_position.ranges[21],board_position.ranges[22],board_position.ranges[23],
			board_position.ranges[24],board_position.ranges[25],board_position.ranges[26],
			board_position.ranges[27],board_position.ranges[28],board_position.ranges[29]);
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "get_board_pos");
	ros::NodeHandle nh;
	ros::Subscriber board_pos_sub = nh.subscribe<sensor_msgs::LaserScan>
	            ("/vision/digit_nws_position", 10, board_pos_cb);
	ros::spin();
	return 0;
}
