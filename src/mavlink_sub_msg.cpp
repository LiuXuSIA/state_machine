/**
* @file     : mavlink_sub_msg.cpp
* @brief    : subscribe mavlink message.
* @author   : libn
* @time     : Sep 21, 2016 9:20:32 PM
*/

#include <ros/ros.h>
#include <state_machine/mavros_test_msg.h>
#include <state_machine/State.h>
//#include <state_machine/ObstaclePosition.h>	/* just for test! */
#include <state_machine/FIXED_TARGET_POSITION_P2M.h>
#include <state_machine/TASK_STATUS_CHANGE_P2M.h>

state_machine::State current_state;
void state_cb(const state_machine::State::ConstPtr& msg){
    current_state = *msg;
}

state_machine::FIXED_TARGET_POSITION_P2M fixed_target_position_p2m_data;
void fixed_target_position_p2m_cb(const state_machine::FIXED_TARGET_POSITION_P2M::ConstPtr& msg){
	fixed_target_position_p2m_data = *msg;
	ROS_INFO("subscribing fixed_target_position_p2m: %5.3f %5.3f %5.3f",
			fixed_target_position_p2m_data.home_lat,
			fixed_target_position_p2m_data.home_lon,
			fixed_target_position_p2m_data.home_alt);
}
//state_machine::ObstaclePosition obstacle_pos;
//void obstacle_position_cb(const state_machine::ObstaclePosition::ConstPtr& msg){
//	obstacle_pos = *msg;
//	ROS_INFO("obstacle_pos: \n"
//			"%f %f %f\n",
//			obstacle_pos.obstacle_x,obstacle_pos.obstacle_y,obstacle_pos.obstacle_z);
//}

state_machine::TASK_STATUS_CHANGE_P2M task_status_change_p2m_data;
void task_status_change_p2m_cb(const state_machine::TASK_STATUS_CHANGE_P2M::ConstPtr& msg){
	task_status_change_p2m_data = *msg;
	ROS_INFO("subscribing task_status_change_p2m: %5.3f %d %d",
				task_status_change_p2m_data.spray_duration,
				task_status_change_p2m_data.task_status,
				task_status_change_p2m_data.loop_value);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "subscribe_msg_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<state_machine::State>("mavros/state", 10, state_cb);

//    ros::Subscriber obstacle_pos_sub = nh.subscribe<state_machine::ObstaclePosition>("mavros/obstacle_position", 10, obstacle_position_cb);

    ros::Subscriber fixed_target_position_p2m_sub = nh.subscribe<state_machine::FIXED_TARGET_POSITION_P2M>("mavros/fixed_target_position_p2m", 10, fixed_target_position_p2m_cb);
    ros::Subscriber task_status_change_p2m_sub = nh.subscribe<state_machine::TASK_STATUS_CHANGE_P2M>("mavros/task_status_change_p2m", 10, task_status_change_p2m_cb);

//    ros::Subscriber task_state_change_sub = nh.subscribe<state_machine::TaskStatusChange>("mavros/task_status_change", 10, task_state_change_cb);

//	ros::Publisher  test_msg_pub = nh.advertise<state_machine::mavros_test_msg>("mavros/mavros_test_msg", 10);
//    ros::Publisher  test_msg_pub = nh.advertise<state_machine::mavros_test_msg>("mavros_test_msg", 10);

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("I was alive.");

//    state_machine::mavros_test_msg msg_test;
//    while(ros::ok()){
//
//		msg_test.test = 3;
//		test_msg_pub.publish(msg_test);
//        ros::spinOnce();
//        rate.sleep();
//    }
    ros::spin();
    return 0;
}
