/*************************************************************************
@file           state_machine_demo.cpp
@date           2018/09/20 18:22
@author         liuxu
@email          liuxu.ccc@gmail.com
@description    a serial communication test
*************************************************************************/

#include <ros/ros.h> 
#include <serial/serial.h> 
#include <std_msgs/String.h> 
#include <std_msgs/Empty.h> 
#include <std_msgs/Float32.h> 
#include <vector> 

using namespace std;

int write_data[4] = {0x80, 0x06, 0x02, 0x78};
int write_data1[4] = {0x80, 0x06, 0x03, 0x77};

vector<uint8_t> data_write(write_data,write_data+4);
vector<uint8_t> data_write1(write_data1,write_data1+4);


serial::Serial ser; 

float dis;
 
void write_callback(const std_msgs::String::ConstPtr& msg) 
{ 
    ROS_INFO_STREAM("Writing to serial port" <<msg->data); 
    ser.write(msg->data); 
} 

int main (int argc, char** argv) 
{ 
    ros::init(argc, argv, "serial_example_node"); 

    ros::NodeHandle nh; 
 
    ros::Subscriber write_sub = nh.subscribe("write", 1000, write_callback);  
    ros::Publisher read_pub = nh.advertise<std_msgs::String>("read", 1000); 

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
    } 
    else 
    { 
        return -1; 
    } 

    ros::Rate loop_rate(50); 
    
    // ser.write(data_write);
    ser.write(data_write1);

    while(ros::ok()) 
    { 
        if(ser.available()){ 
            ROS_INFO_STREAM("Reading from serial port\n"); 
            std_msgs::String result; 
            vector<uint8_t> data_dis;
            int aaa;
            float bbb;
            result.data = ser.read(ser.available()); 
            //ser.read(data_dis,ser.available()); 
            // result.data.erase(0,3);
            // result.data.erase(7,1);
       
            ROS_INFO_STREAM("Read: " << result.data);
            ROS_INFO_STREAM(result.data[0]);
            ROS_INFO_STREAM(result.data[1]);
            ROS_INFO_STREAM(result.data[2]);
            ROS_INFO_STREAM(result.data[3]);
            ROS_INFO_STREAM(result.data[4]);
            ROS_INFO_STREAM(result.data[5]);
            ROS_INFO_STREAM(result.data[6]);
            ROS_INFO_STREAM(result.data[7]);
            ROS_INFO_STREAM(result.data[8]);
            ROS_INFO_STREAM(result.data[9]);
            ROS_INFO_STREAM(result.data[10]);

            ROS_INFO("result.data[0]:%x",result.data[0]);
            ROS_INFO("result.data[1]:%x",result.data[1]);
            ROS_INFO("result.data[2]:%x",result.data[2]);
            ROS_INFO("result.data[3]:%x",result.data[3]);
            ROS_INFO("result.data[4]:%x",result.data[4]);
            ROS_INFO("result.data[5]:%x",result.data[5]);
            ROS_INFO("result.data[6]:%x",result.data[6]);
            ROS_INFO("result.data[7]:%x",result.data[7]);
            ROS_INFO("result.data[8]:%x",result.data[8]);
            ROS_INFO("result.data[9]:%x",result.data[9]);
            ROS_INFO("result.data[10]:%x",result.data[10]);

            aaa = result.data[3];

            ROS_INFO("%d",aaa);

            bbb = (result.data[3]-48)*100+(result.data[4]-48)*10+(result.data[5]-48)+
                  (result.data[7]-48)*0.1+(result.data[8]-48)*0.01+(result.data[9]-48)*0.001;

            ROS_INFO("%f",bbb);
            // dis = atof(result.data.c_str());

            // ROS_INFO_STREAM(data_dis.size());
            // ROS_INFO("%d",data_dis[0]);
            // ROS_INFO("%d",data_dis[1]);
            // ROS_INFO("%d",data_dis[2]);
            //ROS_INFO_STREAM(data_dis[1]);
            //ROS_INFO_STREAM(data_dis[2]);
            // ROS_INFO_STREAM(data_dis[3]);
            // ROS_INFO_STREAM(data_dis[4]);
            // ROS_INFO_STREAM(data_dis[5]);
            // ROS_INFO_STREAM(data_dis[6]);
            // ROS_INFO_STREAM(data_dis[7]);
            // ROS_INFO_STREAM(data_dis[8]);
            // ROS_INFO_STREAM(data_dis[9]);
            // ROS_INFO("%d",data_dis[3]);
            // ROS_INFO("%d",data_dis[4]);
            // ROS_INFO("%d",data_dis[5]);
            // ROS_INFO("%d",data_dis[6]);
            // ROS_INFO("%d",data_dis[7]);
            // ROS_INFO("%d",data_dis[8]);
            // ROS_INFO("%d",data_dis[9]);
            // ROS_INFO("%d",data_dis[10]);
            //ROS_INFO_STREAM(data_dis[10]);
            //dis = result.data[0]*100+result.data[1]*10+result.data[2]+result.data[4]*0.1+result.data[5]*0.01+result.data[6]*0.001;
            //ROS_INFO("%f",dis);
            read_pub.publish(result); 
        } 
        ros::spinOnce(); 
        loop_rate.sleep(); 
    } 
} 