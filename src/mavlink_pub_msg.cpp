/**
* @file     : mavlink_pub_msg.cpp(just for test!)
* @brief    : publish mavlink message.
* @author   : libn
* @time     : Sep 21, 2016 9:20:32 PM
*/

#include <ros/ros.h>
#include <state_machine/FIXED_TARGET_RETURN_M2P.h>
#include <state_machine/OBSTACLE_POSITION_M2P.h>
#include <state_machine/TASK_STATUS_MONITOR_M2P.h>
#include <state_machine/VISION_NUM_SCAN_M2P.h>
#include <state_machine/VISION_ONE_NUM_GET_M2P.h>
#include <state_machine/YAW_SP_CALCULATED_M2P.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "publish_msg_node");
    ros::NodeHandle nh;

//    ros::Subscriber state_sub = nh.subscribe<state_machine::State>("mavros/state", 10, state_cb);
//
//    ros::Subscriber fix_pos_sub = nh.subscribe<state_machine::FixedTargetPosition>("mavros/fixed_target_position", 10, fix_target_pos_cb);

//	ros::Publisher  test_msg_pub = nh.advertise<state_machine::mavros_test_msg>("mavros/mavros_test_msg", 10);
    ros::Publisher  fixed_target_return_m2p_pub  = nh.advertise<state_machine::FIXED_TARGET_RETURN_M2P>("mavros/fixed_target_return_m2p", 10);
    ros::Publisher  obstacle_position_m2p_pub  = nh.advertise<state_machine::OBSTACLE_POSITION_M2P>("mavros/obstacle_position_m2p", 10);
    ros::Publisher  task_status_monitor_m2p_pub  = nh.advertise<state_machine::TASK_STATUS_MONITOR_M2P>("mavros/task_status_monitor_m2p", 10);
    ros::Publisher  vision_num_scan_m2p_pub  = nh.advertise<state_machine::VISION_NUM_SCAN_M2P>("mavros/vision_num_scan_m2p", 10);
    ros::Publisher  vision_one_num_get_m2p_pub  = nh.advertise<state_machine::VISION_ONE_NUM_GET_M2P>("mavros/vision_one_num_get_m2p", 10);
    ros::Publisher  yaw_sp_calculated_m2p_pub  = nh.advertise<state_machine::YAW_SP_CALCULATED_M2P>("mavros/yaw_sp_calculated_m2p", 10);

//    ros::Publisher  task_monitor_pub  = nh.advertise<state_machine::TaskStatusMonitor>("mavros/task_status_monitor", 10);
//
//    ros::Publisher  vision_num_scan_pub  = nh.advertise<state_machine::VisionNumScan>("mavros/vision_num_scan", 10);
//
//    ros::Publisher  vision_one_num_get_pub  = nh.advertise<state_machine::VisionOneNumGet>("mavros/vision_one_num_get", 10);
//
//    ros::Publisher  yaw_sp_pub  = nh.advertise<state_machine::YawSpCalculated>("mavros/yaw_sp_calculated", 10);

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    ROS_INFO("I was alive.");

    state_machine::FIXED_TARGET_RETURN_M2P fixed_target_return_m2p_data;
    state_machine::OBSTACLE_POSITION_M2P obstacle_position_m2p_data;
    state_machine::TASK_STATUS_MONITOR_M2P task_status_monitor_m2p_data;
    state_machine::VISION_NUM_SCAN_M2P vision_num_scan_m2p_data;
	state_machine::VISION_ONE_NUM_GET_M2P vision_one_num_get_m2p_data;
	state_machine::YAW_SP_CALCULATED_M2P yaw_sp_calculated_m2p_data;

    while(ros::ok())
    {


    	fixed_target_return_m2p_data.home_lat= 1.0f;
    	fixed_target_return_m2p_data.home_lon= 1.0f;
    	fixed_target_return_m2p_data.home_alt= 1.0f;
		fixed_target_return_m2p_pub.publish(fixed_target_return_m2p_data);
		ROS_INFO("publishing fixed_target_return_m2p: %f\t%f\t%f\t",
				fixed_target_return_m2p_data.home_lat,
				fixed_target_return_m2p_data.home_lon,
				fixed_target_return_m2p_data.home_alt);

		obstacle_position_m2p_data.obstacle_x = 2.0f;
		obstacle_position_m2p_data.obstacle_y = 2.0f;
		obstacle_position_m2p_data.obstacle_z = 2.0f;
		obstacle_position_m2p_data.obstacle_valid = true;
		obstacle_position_m2p_pub.publish(obstacle_position_m2p_data);
		ROS_INFO("publishing obstacle_position_m2p: %f\t%f\t%f\t%d",
				obstacle_position_m2p_data.obstacle_x,
				obstacle_position_m2p_data.obstacle_y,
				obstacle_position_m2p_data.obstacle_z,
				obstacle_position_m2p_data.obstacle_valid);

		task_status_monitor_m2p_data.spray_duration = 0.3f;
		task_status_monitor_m2p_data.task_status = 0;
		task_status_monitor_m2p_data.loop_value = 0;
		task_status_monitor_m2p_data.target_lat = 3.0f;
		task_status_monitor_m2p_data.target_lon = 3.0f;
		task_status_monitor_m2p_data.target_alt = 3.0f;
		task_status_monitor_m2p_pub.publish(task_status_monitor_m2p_data);
		ROS_INFO("publishing task_status_monitor_m2p: %f %d %d %f %f %f",
				task_status_monitor_m2p_data.spray_duration,
				task_status_monitor_m2p_data.task_status,
				task_status_monitor_m2p_data.loop_value,
				task_status_monitor_m2p_data.target_lat,
				task_status_monitor_m2p_data.target_lon,
				task_status_monitor_m2p_data.target_alt);

		vision_num_scan_m2p_data.board_num = 0;
		vision_num_scan_m2p_data.board_x = 4.0f;
		vision_num_scan_m2p_data.board_y = 4.0f;
		vision_num_scan_m2p_data.board_z = 4.0f;
		vision_num_scan_m2p_data.board_valid = true;
		vision_num_scan_m2p_pub.publish(vision_num_scan_m2p_data);
		ROS_INFO("publishing vision_num_scan_m2p: %d %f %f %f %d",
				vision_num_scan_m2p_data.board_num,
				vision_num_scan_m2p_data.board_x,
				vision_num_scan_m2p_data.board_y,
				vision_num_scan_m2p_data.board_z,
				vision_num_scan_m2p_data.board_valid);

		vision_one_num_get_m2p_data.loop_value = 0;
		vision_one_num_get_m2p_data.num = 5;
		vision_one_num_get_m2p_pub.publish(vision_one_num_get_m2p_data);
		ROS_INFO("publishing vision_one_num_get_m2p: %d %d",
				vision_one_num_get_m2p_data.loop_value,
				vision_one_num_get_m2p_data.num);

		yaw_sp_calculated_m2p_data.yaw_sp = 0.6f;
		yaw_sp_calculated_m2p_pub.publish(yaw_sp_calculated_m2p_data);
		ROS_INFO("publishing yaw_sp_calculated_m2p: %f",
				yaw_sp_calculated_m2p_data.yaw_sp);

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
