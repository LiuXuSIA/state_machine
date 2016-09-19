/**
* @file     : get_board_position.cpp
* @brief    : publish the position of 10 boards.
* @author   : libn
* @time     : Sep 17, 2016 9:50:58 PM
*/

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <state_machine/DrawingBoard.h>
#include <state_machine/DrawingBoard10.h>
#include <geometry_msgs/PoseStamped.h>

ros::Publisher DrawingBoard_Position_pub;

ros::Time last_request;

// local position msg callback function
geometry_msgs::PoseStamped current_pos;
void pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    current_pos = *msg;
}

/* send indexed setpoint. -libn <Aug 15, 2016 9:00:02 AM> */
state_machine::DrawingBoard board;
state_machine::DrawingBoard10 board10;

sensor_msgs::LaserScan board_scan;
void board_pos_cb(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	board_scan = *msg;
    int amout = board_scan.ranges.size()/4;
    int num;
    for ( int i = 0; i < amout; ++i )
    {
        num = (int)board_scan.ranges[i*4];  /* No. of board detected. -libn */
        if(num == 11)	break;	/* incomplete rectangle detected. -libn */
        else if(num >9 || num <0)
        {
        	ROS_INFO("board num error!");
        	break;
        }

//        ROS_INFO("data received:\n"
//            		"num = %d "
//            		"x = %f "
//            		"y = %f "
//            		"z = %f",
//					num,board_scan.ranges[i*4 + 1],board_scan.ranges[i*4 + 1],board_scan.ranges[i*4 + 1]);

        board10.drawingboard[num].num = num;
        board10.drawingboard[num].x = board_scan.ranges[i*4 + 1] + current_pos.pose.position.x;
        board10.drawingboard[num].y = board_scan.ranges[i*4 + 2] + current_pos.pose.position.y;
        board10.drawingboard[num].z = board_scan.ranges[i*4 + 3] + current_pos.pose.position.z;
        board10.drawingboard[num].valid = true;

    }

    DrawingBoard_Position_pub.publish(board10);

    if((ros::Time::now() - last_request) > ros::Duration(0.5))	/* display with delay of 0.5s -libn */
    {
    	ROS_INFO("current pos:\n"
    			"x = %5.3f y = %5.3f z = %5.3f\n",
				current_pos.pose.position.x,current_pos.pose.position.y,current_pos.pose.position.z);
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
    	last_request = ros::Time::now();
    }

}


int main(int argc, char **argv)
{
	ROS_INFO("I was alive.");
	ros::init(argc, argv, "get_board_pos");
	ros::NodeHandle nh;

    DrawingBoard_Position_pub = nh.advertise<state_machine::DrawingBoard10>("DrawingBoard_Position10", 1);

	ros::Subscriber board_pos_sub = nh.subscribe<sensor_msgs::LaserScan>
	            ("/vision/digit_nws_position", 10, board_pos_cb);
	board10.drawingboard.resize(10);		/* MUST! -libn */

	/* get pixhawk's local position. -libn */
	ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, pos_cb);


	for(int i = 0; i < 10; i++)
	{
        board10.drawingboard[i].num = 66;
        board10.drawingboard[i].x = 0.0f;
        board10.drawingboard[i].y = 0.0f;
        board10.drawingboard[i].z = 0.0f;
        board10.drawingboard[i].valid = false;

	}

	last_request = ros::Time::now();

	ros::spin();
	return 0;
}
