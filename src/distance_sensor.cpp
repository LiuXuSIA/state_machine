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
#include <state_machine/Distance.h> 
#include <vector> 

using namespace std;

//distance sensor
const int write_data[4] = {0x80, 0x06, 0x02, 0x78}; //rad once
const int write_data1[4] = {0x80, 0x06, 0x03, 0x77};//read continuous

const vector<uint8_t> read_once(write_data,write_data+4);
const vector<uint8_t> read_continuous(write_data1,write_data1+4);

serial::Serial ser; 

int main (int argc, char** argv) 
{ 
    ros::init(argc, argv, "distance_sensor_node"); 

    ros::NodeHandle nh; 
 
    ros::Publisher distance_pub = nh.advertise<state_machine::Distance>("distance", 10); 

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
    
    ser.write(read_continuous);

    while(ros::ok()) 
    { 
        if(ser.available())
        { 
            ROS_INFO_STREAM("Reading:"); 
            std_msgs::String result; 
            state_machine::Distance distance;
            vector<uint8_t> data_dis;
            ser.read(data_dis,ser.available()); 

            distance.distance = (data_dis[3]-48)*100+(data_dis[4]-48)*10+(data_dis[5]-48)+
                       (data_dis[7]-48)*0.1+(data_dis[8]-48)*0.01+(data_dis[9]-48)*0.001;
            ROS_INFO("distance:%f",distance.distance);
            distance_pub.publish(distance); 
        } 

        ros::spinOnce(); 
        rate.sleep(); 
    } 
} 