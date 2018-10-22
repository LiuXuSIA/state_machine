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

//custom distance->mavros
#include <state_machine/Distance.h> 
#include <state_machine/Distance_Measure_Enable.h>

/***************************function declare****************************/
void state_machine_fun(void);

/***************************variable definition*************************/

//state_machine state
static const int initial = 1;
static const int grab_status_judge = 10;
static const int leave = 17;
static const int place_status_judge = 19;

//mission 
int loop = 0;
int current_pos_state = initial;

//time
ros::Time last_time;

state_machine::TASK_STATUS_MONITOR_M2P task_status_monitor;
state_machine::Distance_Measure_Enable measure_enable;

ros::Publisher distance_measure_enable_pub;

state_machine::State current_state;
void state_cb(const state_machine::State::ConstPtr& msg)
{
    current_state = *msg;
}

state_machine::Distance distance;
void distance_cb(const state_machine::Distance::ConstPtr& msg)
{
    distance = *msg;
    ROS_INFO("distance:%f",distance.distance);
}

/*****************************main function*****************************/
int main(int argc, char **argv)
{
    ros::init(argc, argv, "offboard_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<state_machine::State>("mavros/state",10,state_cb);
    ros::Publisher task_status_pub = nh.advertise<state_machine::TASK_STATUS_MONITOR_M2P>("mavros/task_status_monitor_m2p",10);
    ros::Subscriber distance_sub = nh.subscribe<state_machine::Distance>("distance",10,distance_cb);
    distance_measure_enable_pub = nh.advertise<state_machine::Distance_Measure_Enable>("distance_measure_enable",10);

    ros::Rate rate(10.0);

    measure_enable.measure_enable = 0;        
 
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
        task_status_monitor.sensor_distance = distance.distance;
        task_status_pub.publish(task_status_monitor);

        distance_measure_enable_pub.publish(measure_enable);
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
            if(ros::Time::now() - last_time > ros::Duration(5.0))
            {
                current_pos_state = grab_status_judge;
                ROS_INFO("measure start!!!");
                last_time = ros::Time::now();
            }
        }
        break;
        case grab_status_judge:
        {
            measure_enable.measure_enable = 1;
    
            if(ros::Time::now() - last_time > ros::Duration(5.0))
            {
                measure_enable.measure_enable = 0; 
                ROS_INFO("measure stop!!!");
                current_pos_state = leave;
                last_time = ros::Time::now();
            }
        }
        break;
        case leave:
        break;
    }
            
}