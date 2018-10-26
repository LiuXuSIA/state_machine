/*************************************************************************
@file           distance_sensor.cpp
@date           2018/09/24 09:20
@author         liuxu
@email          liuxu.ccc@gmail.com
@description    distance_sensor_node
*************************************************************************/

#include <ros/ros.h> 
#include <serial/serial.h> 
#include <std_msgs/String.h> 
#include <std_msgs/Empty.h> 
#include <std_msgs/Float32.h>
#include <state_machine/Vision_Position_Multiple.h>

#include <state_machine/Attitude.h>
#include <geometry_msgs/PoseStamped.h>
#include "math.h"


serial::Serial ser; 

state_machine::Vision_Position_Multiple vision_position_multiple;

float R[3][3] = {0};
float body_pose_x_dis,body_pose_y_dis,body_pose_z_dis;
float body_pose_x_deep,body_pose_y_deep,body_pose_z_deep;

geometry_msgs::PoseStamped current_position;
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    current_position = *msg;
}

state_machine::Attitude current_attitude;
void attitude_cb(const state_machine::Attitude::ConstPtr& msg)
{
    current_attitude = *msg;

    R[0][0] = cos(current_attitude.pitch) * cos(current_attitude.yaw);
    R[0][1] = sin(current_attitude.roll) * sin(current_attitude.pitch) * cos(current_attitude.yaw) -
              cos(current_attitude.roll) * sin(current_attitude.yaw);
    R[0][2] = cos(current_attitude.roll) * sin(current_attitude.pitch) * cos(current_attitude.yaw) +
              sin(current_attitude.roll) * sin(current_attitude.yaw);

    R[1][0] = cos(current_attitude.pitch) * sin(current_attitude.yaw);
    R[1][1] = sin(current_attitude.roll) * sin(current_attitude.pitch) * sin(current_attitude.yaw) +
              cos(current_attitude.roll) * cos(current_attitude.yaw);
    R[1][2] = cos(current_attitude.roll) * sin(current_attitude.pitch) * sin(current_attitude.yaw) -
              sin(current_attitude.roll) * cos(current_attitude.yaw);

    R[2][0] = -sin(current_attitude.pitch);
    R[2][1] = sin(current_attitude.roll) * cos(current_attitude.pitch);
    R[2][2]= cos(current_attitude.roll) * cos(current_attitude.pitch);

}

int main (int argc, char** argv) 
{ 
    ros::init(argc, argv, "vision_position_node"); 

    ros::NodeHandle nh;  
    
    //for coordinate exchange
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose",10,pose_cb);
    ros::Subscriber attitude_sub = nh.subscribe<state_machine::Attitude>("mavros/attitude",10,attitude_cb);
    //vision_position
    ros::Publisher vision_position_pub = nh.advertise<state_machine::Vision_Position_Multiple>("vision_position", 10); 

    ros::Rate rate(20.0); 

    try 
    { 
        ser.setPort("/dev/ttyUSB2"); 
        ser.setBaudrate(9600); 
        serial::Timeout to = serial::Timeout::simpleTimeout(1000); 
        ser.setTimeout(to); 
        ser.open(); 
    } 
    catch (serial::IOException& e) 
    { 
        ROS_ERROR_STREAM("Unable to open port "); 
        return -1; 
    } 

    if(ser.isOpen()) 
    { 
        ROS_INFO_STREAM("Serial Port initialized"); 
        ROS_INFO_STREAM("Waitting for vision information...");
    } 
    else 
    { 
        return -1; 
    } 

    while(ros::ok()) 
    { 
        if(ser.available())
        { 
            ros::spinOnce();

            ROS_INFO_STREAM("Reading:"); 
            std_msgs::String position_data; 
            float position_x_min_distance,position_y_min_distance,position_z_min_distance;
            float position_x_min_deepth,position_y_min_deepth,position_z_min_deepth;
            std_msgs::String position_x_temp_dis,position_y_temp_dis,position_z_temp_dis;
            std_msgs::String position_x_temp_deep,position_y_temp_deep,position_z_temp_deep;
            position_data.data = ser.read(ser.available()); 
            ROS_INFO_STREAM("Read: " << position_data.data);

            if(position_data.data.substr(0,4) == "SSSS" &&
               position_data.data.substr(position_data.data.length() - 4,4) == "EEEE")
            {
                position_x_temp_dis.data = position_data.data.substr(5,5);
                position_y_temp_dis.data = position_data.data.substr(11,5);
                position_z_temp_dis.data = position_data.data.substr(17,5);

                position_x_temp_deep.data = position_data.data.substr(25,5);
                position_y_temp_deep.data = position_data.data.substr(31,5);
                position_z_temp_deep.data = position_data.data.substr(37,5);

                position_x_min_deepth = atof(position_x_temp_deep.data.substr(1,4).c_str());
                position_y_min_deepth = atof(position_y_temp_deep.data.substr(1,4).c_str());
                position_z_min_deepth = atof(position_z_temp_deep.data.substr(1,4).c_str());

                position_x_min_distance = atof(position_x_temp_dis.data.substr(1,4).c_str());
                position_y_min_distance = atof(position_y_temp_dis.data.substr(1,4).c_str());
                position_z_min_distance = atof(position_z_temp_dis.data.substr(1,4).c_str());

                if(position_x_temp_deep.data[0] == '0')
                {
                    position_x_min_deepth = -position_x_min_deepth; 
                }
                if(position_y_temp_deep.data[0] == '0')
                {
                    position_y_min_deepth = -position_y_min_deepth; 
                }
                if(position_z_temp_deep.data[0] == '0')
                {
                    position_z_min_deepth = -position_z_min_deepth; 
                }

                if(position_x_temp_dis.data[0] == '0')
                {
                    position_x_min_distance = -position_x_min_distance; 
                }
                if(position_y_temp_dis.data[0] == '0')
                {
                    position_y_min_distance = -position_y_min_distance; 
                }
                if(position_z_temp_dis.data[0] == '0')
                {
                    position_z_min_distance = -position_z_min_distance; 
                }

                ROS_INFO("deep_x:%f",position_x_min_deepth);
                ROS_INFO("deep_y:%f",position_y_min_deepth);
                ROS_INFO("deep_z:%f",position_z_min_deepth);

                ROS_INFO("dis_x:%f",position_x_min_distance);
                ROS_INFO("dis_y:%f",position_y_min_distance);
                ROS_INFO("dis_z:%f",position_z_min_distance);

                body_pose_x_deep = -position_x_min_deepth;
                body_pose_y_deep = -position_y_min_deepth;
                body_pose_z_deep = position_z_min_deepth;

                body_pose_x_dis = -position_x_min_distance;
                body_pose_y_dis = -position_y_min_distance;
                body_pose_z_dis = position_z_min_distance;

                vision_position_multiple.x_deep = (R[0][0] * body_pose_x_deep + R[0][1] * body_pose_y_deep + R[0][2] * body_pose_z_deep)/1000 + current_position.pose.position.y;
                vision_position_multiple.y_deep = (R[1][0] * body_pose_x_deep + R[1][1] * body_pose_y_deep + R[1][2] * body_pose_z_deep)/1000 + current_position.pose.position.x;
                vision_position_multiple.z_deep = body_pose_z_deep/1000;

                vision_position_multiple.x_dis = (R[0][0] * body_pose_x_dis + R[0][1] * body_pose_y_dis + R[0][2] * body_pose_z_dis)/1000 + current_position.pose.position.y;
                vision_position_multiple.y_dis = (R[1][0] * body_pose_x_dis + R[1][1] * body_pose_y_dis + R[1][2] * body_pose_z_dis)/1000 + current_position.pose.position.x;
                vision_position_multiple.z_dis = body_pose_z_dis/1000;

                vision_position_pub.publish(vision_position_multiple);
            }
        } 

        rate.sleep(); 
    } 
} 