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
#include <state_machine/Distance_Measure_Enable.h>

using namespace std;

//distance sensor
const int write_data[4] = {0xe8, 0x02, 0xbc}; 

const vector<uint8_t> read_once(write_data,write_data + 3);

serial::Serial ser; 



state_machine::Distance_Measure_Enable distance_measure;
void distance_measure_enable_cb(const state_machine::Distance_Measure_Enable::ConstPtr& msg)
{
    distance_measure = *msg;
    ROS_INFO("distance:%d",distance_measure.measure_enable);
}

int main (int argc, char** argv) 
{ 
    ros::init(argc, argv, "distance_sensor_node"); 

    ros::NodeHandle nh; 
 
    ros::Publisher distance_pub = nh.advertise<state_machine::Distance>("distance", 10); 
    ros::Subscriber distance_measure_enable_sub = nh.subscribe<state_machine::Distance_Measure_Enable>("distance_measure_enable",10,distance_measure_enable_cb);

    ros::Rate rate(10.0); 

    try 
    { 
        ser.setPort("/dev/ttyUSB0"); 
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
        ROS_INFO_STREAM("Waitting for distance mersure...");
    } 
    else 
    { 
        return -1; 
    } 

    while(ros::ok()) 
    { 
        //if(distance_measure.measure_enable == 1)
        {
            if(ser.available())
            { 
                ROS_INFO_STREAM("Reading:"); 
                static std_msgs::String result; 
                static state_machine::Distance distance;
                vector<uint8_t> data_dis;
                static float last_distance;
                ser.read(data_dis,ser.available()); 
            
                distance.distance = (data_dis[0] * 256 + data_dis[1]) * 0.001;

                ROS_INFO("distance:%f",distance.distance);

                distance_pub.publish(distance); 
            } 
            else
            {
               ser.write(read_once);
            }

        }
        ros::spinOnce(); 
        rate.sleep(); 
    } 
} 