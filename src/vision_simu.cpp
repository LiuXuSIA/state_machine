/*************************************************************************
@file           state_machine_demo.cpp
@date           2018/08/20 16:25
@author         liuxu
@email          liuxu.ccc@gmail.com
@description    a simple state machine for the drone race in 2018
                takeoff-->A-->hover-->B-->hover-->C-->hover-->landing
*************************************************************************/

#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <geometry_msgs/PoseStamped.h>

ros::Publisher car1_num_pub;
ros::Publisher car2_num_pub;
ros::Publisher car3_num_pub;

ros::Publisher car1_position_pub;
ros::Publisher car2_position_pub;
ros::Publisher car3_position_pub;

std_msgs::Int8 car1_num;
std_msgs::Int8 car2_num;
std_msgs::Int8 car3_num;

geometry_msgs::PoseStamped car1_position;
geometry_msgs::PoseStamped car2_position;
geometry_msgs::PoseStamped car3_position;

/*****************************main function*****************************/
int main(int argc, char **argv)
{
    ros::init(argc, argv, "vision_simu");
    ros::NodeHandle nh;

    car1_num.data = -1;
    car2_num.data = -1;
    car3_num.data = -1;

    car1_position.pose.position.x = -1;
    car1_position.pose.position.y = -1;
    car1_position.pose.position.z = -1;

    car2_position.pose.position.x = -1;
    car2_position.pose.position.y = -1;
    car2_position.pose.position.z = -1;

    car3_position.pose.position.x = -1;
    car3_position.pose.position.y = -1;
    car3_position.pose.position.z = -1;

    car1_num_pub = nh.advertise<std_msgs::Int8>("car1_num",10);
    car2_num_pub = nh.advertise<std_msgs::Int8>("car2_num",10);
    car3_num_pub = nh.advertise<std_msgs::Int8>("car3_numl",10);

    car1_position_pub = nh.advertise<geometry_msgs::PoseStamped>("car1_pos",10);
    car2_position_pub = nh.advertise<geometry_msgs::PoseStamped>("car2_pos",10);
    car3_position_pub = nh.advertise<geometry_msgs::PoseStamped>("car3_pos",10);

    ros::Rate rate(10.0);

    while(ros::ok())
    {   
        car1_num_pub.publish(car1_num);
        car2_num_pub.publish(car2_num);
        car3_num_pub.publish(car3_num);

        car1_position_pub.publish(car1_position);
        car2_position_pub.publish(car2_position);
        car3_position_pub.publish(car3_position);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
