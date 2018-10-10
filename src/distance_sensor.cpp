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
const int write_data2[5] = {0xFA, 0x04, 0x08, 0x01,0xF9};//from top
const int write_data3[5] = {0xFA, 0x04, 0x09, 0x0A,0xEF};//set scale to 10
const int write_data4[5] = {0xFA, 0x04, 0x0A, 0x14,0xE4};//set freq to 20
const int write_data5[5] = {0xFA, 0x04, 0x0D, 0x01,0xF4};//start at once when electrify
const int write_data6[5] = {0x80, 0x06, 0x05, 0x01,0x74};//start laser



const vector<uint8_t> read_once(write_data,write_data+4);
const vector<uint8_t> read_continuous(write_data1,write_data1+4);
const vector<uint8_t> dis_from_top(write_data2,write_data2+5);
const vector<uint8_t> set_scale(write_data3,write_data3+5);
const vector<uint8_t> set_freq(write_data4,write_data4+5);
const vector<uint8_t> start_electrify(write_data5,write_data5+5);
const vector<uint8_t> start_laser(write_data6,write_data6+5);

serial::Serial ser; 

int main (int argc, char** argv) 
{ 
    ros::init(argc, argv, "distance_sensor_node"); 

    ros::NodeHandle nh; 
 
    ros::Publisher distance_pub = nh.advertise<state_machine::Distance>("distance", 10); 

    ros::Rate rate(20.0); 

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
    
    //ser.write(start_electrify);
    //ser.write(start_laser);
    //ser.write(dis_from_top);
    //ser.write(set_scale);
    //ser.write(set_freq);
    ser.write(read_continuous);


    while(ros::ok()) 
    { 
        if(ser.available())
        { 
            ROS_INFO_STREAM("Reading:"); 
            static std_msgs::String result; 
            static state_machine::Distance distance;
            vector<uint8_t> data_dis;
            static float last_distance;
            ser.read(data_dis,ser.available()); 
            //result.data = ser.read(ser.available()); 
            //ROS_INFO_STREAM("Read: " << result.data);
            
            distance.distance = (data_dis[3]-48)*100+(data_dis[4]-48)*10+(data_dis[5]-48)+
                        (data_dis[7]-48)*0.1+(data_dis[8]-48)*0.01+(data_dis[9]-48)*0.001;

            if(abs(distance.distance - last_distance) > 3.0) 
                distance.distance = last_distance;
  
            last_distance = distance.distance;
            ROS_INFO("distance:%f",distance.distance);
            //distance_pub.publish(distance); 
        } 

        ros::spinOnce(); 
        rate.sleep(); 
    } 
} 