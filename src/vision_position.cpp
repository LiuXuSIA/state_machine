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


serial::Serial ser; 

state_machine::Vision_Position_Raw vision_position_raw;

int main (int argc, char** argv) 
{ 
    ros::init(argc, argv, "vision_position_node"); 

    ros::NodeHandle nh;  

    ros::Publisher vision_position_pub = nh.advertise<state_machine::Vision_Position_Raw>("vision_position", 10); 

    ros::Rate rate(10.0); 

    try 
    { 
        ser.setPort("/dev/ttyUSB1"); 
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
    } 
    else 
    { 
        return -1; 
    } 

    while(ros::ok()) 
    { 
        if(ser.available())
        { 
            ROS_INFO_STREAM("Reading:"); 
            std_msgs::String position_data; 
            float position_x,position_y,position_z,position_d;
            position_data.data = ser.read(ser.available()); 
            ROS_INFO_STREAM("Read: " << position_data.data);

            if(position_data.data.substr(0,4) == "SSSS" &&
                position_data.data.substr(27,4) == "EEEE")
            {
                position_x = atof(position_data.data.substr(5,5).c_str());
                position_y = atof(position_data.data.substr(11,5).c_str());
                position_z = atof(position_data.data.substr(17,5).c_str());
                position_d = atof(position_data.data.substr(23,4).c_str());

                ROS_INFO("x:%f",position_x);
                ROS_INFO("y:%f",position_y);
                ROS_INFO("z:%f",position_z);
                ROS_INFO("d:%f",position_d);

                vision_position_raw.x = position_x/1000;
                vision_position_raw.y = position_y/1000;
                vision_position_raw.z = position_z/1000;

                vision_position_pub.publish(vision_position_raw);
            }
        } 

        ros::spinOnce(); 
        rate.sleep(); 
    } 
} 