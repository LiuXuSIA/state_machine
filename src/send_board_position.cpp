/**
* @file     : send_board_position.cpp
* @brief    : publish the position of 10 boards(just for test).	-> Not needed any more!
* @author   : libn
* @time     : Sep 18, 2016 10:28:02 PM
*/
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

ros::Publisher vision_digit_position_publisher;

int main(int argc, char **argv)
{
	ROS_INFO("I am alive.");
	ros::init(argc, argv, "send_board_pos");
	ros::NodeHandle nh;
	vision_digit_position_publisher = nh.advertise<sensor_msgs::LaserScan>("vision/digit_nws_position", 1);
	sensor_msgs::LaserScan digits_position;
	int amout = 3;
	digits_position.ranges.resize(amout*4);

	ros::Rate rate(10);		/* 10Hz. -libn <Aug 11, 2016 9:28:08 AM> */

	for ( int i = 0; i < amout; ++i )
	{
		digits_position.ranges[i*4] = (float)i;
		digits_position.ranges[i*4 + 1] = (float)i;
		digits_position.ranges[i*4 + 2] = (float)i;
		digits_position.ranges[i*4 + 3] = (float)i;
	}

	while(ros::ok())
	{
		vision_digit_position_publisher.publish(digits_position);
		ros::spinOnce();	/* refresh data subscriber. -libn <Aug 11, 2016 9:28:33 AM> */
		rate.sleep();
	}

	return 0;
}
