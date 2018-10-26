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
#include <state_machine/Vision_Position_Raw.h>

#include <state_machine/Attitude.h>
#include <geometry_msgs/PoseStamped.h>
#include "math.h"


serial::Serial ser; 

state_machine::Vision_Position_Raw vision_position_raw;

float R[3][3] = {0};
float body_pose_x,body_pose_y,body_pose_z;

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
    ros::Publisher vision_position_pub = nh.advertise<state_machine::Vision_Position_Raw>("vision_position", 10); 

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
            float position_x,position_y,position_z;
            std_msgs::String position_x_temp,position_y_temp,position_z_temp;
            position_data.data = ser.read(ser.available()); 
            ROS_INFO_STREAM("Read: " << position_data.data);

            if(position_data.data.substr(0,4) == "SSSS" &&
               position_data.data.substr(position_data.data.length() - 4,4) == "EEEE")
            {
                position_x_temp.data = position_data.data.substr(5,5);
                position_y_temp.data = position_data.data.substr(11,5);
                position_z_temp.data = position_data.data.substr(17,5);

                position_x = atof(position_x_temp.data.substr(1,4).c_str());
                position_y = atof(position_y_temp.data.substr(1,4).c_str());
                position_z = atof(position_z_temp.data.substr(1,4).c_str());

                if(position_x_temp.data[0] == '0')
                {
                    position_x = -position_x; 
                }
                if(position_y_temp.data[0] == '0')
                {
                    position_y = -position_y; 
                }
                if(position_z_temp.data[0] == '0')
                {
                    position_z = -position_z; 
                }

                ROS_INFO("x:%f",position_x);
                ROS_INFO("y:%f",position_y);
                ROS_INFO("z:%f",position_z);

                body_pose_x = -position_x;
                body_pose_y = -position_y;
                body_pose_z = position_z;

                vision_position_raw.x = (R[0][0] * body_pose_x + R[0][1] * body_pose_y + R[0][2] * body_pose_z)/1000 + current_position.pose.position.y;
                vision_position_raw.y = (R[1][0] * body_pose_x + R[1][1] * body_pose_y + R[1][2] * body_pose_z)/1000 + current_position.pose.position.x;
                vision_position_raw.z = position_z/1000;

                vision_position_pub.publish(vision_position_raw);
            }
        } 

        rate.sleep(); 
    } 
} 