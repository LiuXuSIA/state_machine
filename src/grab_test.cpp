/*************************************************************************
@file           grab_test.cpp
@date           2018/10/02 
@author         liuxu
@email          liuxu.ccc@gmail.com
@description    grab_test  grab->10s->release
*************************************************************************/

#include <ros/ros.h>
#include <state_machine/State.h>
#include <state_machine/TASK_STATUS_MONITOR_M2P.h>

/***************************function declare****************************/
void state_machine_fun(void);

/***************************variable definition*************************/

//state_machine state
static const int initial = 1;
static const int grab = 10;
static const int release = 17;

//mission 
int loop = 0;
int current_pos_state = initial;

//time
ros::Time last_time;

state_machine::TASK_STATUS_MONITOR_M2P task_status_monitor;

state_machine::State current_state;
void state_cb(const state_machine::State::ConstPtr& msg)
{
    current_state = *msg;
}

/*****************************main function*****************************/
int main(int argc, char **argv)
{
    ros::init(argc, argv, "offboard_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<state_machine::State>("mavros/state",10,state_cb);
    ros::Publisher task_status_pub = nh.advertise<state_machine::TASK_STATUS_MONITOR_M2P>("mavros/task_status_monitor_m2p",10);


    ros::Rate rate(10.0);

    while(ros::ok() && !current_state.connected)
    {
    	ros::spinOnce();
    	rate.sleep();
    }

    ROS_INFO("Connect successfully!!");
 
    last_time = ros::Time::now();

	while(ros::ok())
	{
        state_machine_fun();

		ros::spinOnce();
		rate.sleep();

        task_status_monitor.task_status = current_pos_state;
        task_status_monitor.loop_value = loop;
        task_status_monitor.target_x = 0;
        task_status_monitor.target_y = 0;
        task_status_monitor.target_z = 0;
        task_status_monitor.sensor_distance = 0;
        task_status_pub.publish(task_status_monitor);
	}

	return 0;
}

/***********************************state_machine function***********************************/
void state_machine_fun(void)
{
    switch(current_pos_state)
    {
        case initial:
        {
            if(ros::Time::now() - last_time > ros::Duration(10.0))
            {
                current_pos_state = grab;
                last_time = ros::Time::now();
            }
        }
        break;
        case grab:
        {
            if(ros::Time::now() - last_time > ros::Duration(7.0))
            {
                current_pos_state = release;
                last_time = ros::Time::now();
            }
        }
        break;
        case release:
        break;
    }
            
}