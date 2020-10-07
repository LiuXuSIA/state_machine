/*************************************************************************
@file           state_machine_demo.cpp
@date           2018/08/20 16:25
@author         liuxu
@email          liuxu.ccc@gmail.com
@description    a simple state machine for the drone race in 2018
                takeoff-->A-->hover-->B-->hover-->C-->hover-->landing
*************************************************************************/

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <state_machine/CommandTOL.h>
#include <state_machine/CommandBool.h>
#include <state_machine/State.h>
#include <state_machine/attributeStatus_F2L.h>
#include <state_machine/requestCommand_L2F.h>
#include <std_msgs/String.h>
#include <state_machine/positionXY.h>
#include <state_machine/SetMode.h>
#include "math.h"
#include <state_machine/Attitude.h>
#include <state_machine/GPS_Status.h>
#include <sensor_msgs/BatteryState.h>
#include <state_machine/taskStatusMonitor.h>


/***************************function declare****************************/

void state_machine_fun(void);
double Distance_of_Two(double x1, double x2, double y1, double y2, double z1, double z2);

/***************************variable definition*************************/
//position  ENU
geometry_msgs::PoseStamped position_A;
geometry_msgs::PoseStamped position_B;
geometry_msgs::PoseStamped position_C;
geometry_msgs::PoseStamped position_next;
geometry_msgs::PoseStamped hover_position;

geometry_msgs::PoseStamped pose_pub;  //ENU
geometry_msgs::TwistStamped vel_pub;

//for take off
ros::Publisher local_pos_pub;
ros::Publisher local_vel_pub;
ros::Publisher takeOffStatus_pub;
ros::Publisher communication_result_pub;
ros::Publisher emergency_status_pub;
ros::Publisher current_position_pub;
//added for gs
ros::Publisher current_pose_pub;
ros::Publisher current_attitude_pub;
ros::Publisher battery_staus_pub;
ros::Publisher current_mode_pub;
ros::Publisher current_gps_status_pub;
ros::Publisher task_status_pub;

state_machine::attributeStatus_F2L takeOffStatus;
state_machine::attributeStatus_F2L communicationStatus;
state_machine::attributeStatus_F2L emergencyStatus;
// takeOffStatus.UAV_index = 0;
// takeOffStatus.value = 0;
//bool velocity_control_enable = true;


//state_machine state
//every state need target position

static const int takeoff = 1;
static const int position_A_go = 2;
static const int position_A_hover = 3;
static const int position_tracking = 4;
static const int current_position_hover = 5;
static const int land = 6;

//mission 
int loop = 0;
int current_pos_state = takeoff;
std_msgs::String current_position_state;
state_machine::taskStatusMonitor task_status_monitor;

//time
ros::Time last_time;

//velocity send
bool velocity_control_enable = true;

double take_off_height;

//get the home position before takeoff
bool get_home_position_enable = false;

//land
ros::ServiceClient land_client;
ros::Time landing_last_request;

ros::ServiceClient arming_client;
ros::ServiceClient set_mode_client_offboard;
ros::ServiceClient set_mode_client_posctl;
ros::Time last_request;

/*************************constant defunition***************************/

#define ASCEND_VELOCITY     1.0
#define LOCATE_ACCURACY     0.5

/***************************callback function definition***************/
state_machine::State current_state;
std_msgs::String current_mode;
void state_cb(const state_machine::State::ConstPtr& msg)
{
    current_state = *msg;
    current_mode.data = current_state.mode;
    current_mode_pub.publish(current_mode);
}

geometry_msgs::PoseStamped current_position;
bool home_position_gotten = false;
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    current_position = *msg;
    current_pose_pub.publish(current_position);
    if(get_home_position_enable == true)
    {
        //position of A
        position_A.pose.position.x = current_position.pose.position.x;
        position_A.pose.position.y = current_position.pose.position.y;
        position_A.pose.position.z = current_position.pose.position.z+5;
        //position of B
        position_B.pose.position.x = position_A.pose.position.x;
        position_B.pose.position.y = position_A.pose.position.y+5;
        position_B.pose.position.z = position_A.pose.position.z;
        //position of C
        position_C.pose.position.x = position_A.pose.position.x+5;
        position_C.pose.position.y = position_A.pose.position.y+5;
        position_C.pose.position.z = position_A.pose.position.z;

        take_off_height = position_A.pose.position.z-2;

        ROS_INFO("gotten the home position.");
        ROS_INFO("the x of home:%f", position_A.pose.position.x);
        ROS_INFO("the y of home:%f", position_A.pose.position.y);
        ROS_INFO("the z of home:%f", position_A.pose.position.z);

        get_home_position_enable = false;
        home_position_gotten = true;
    }
}

geometry_msgs::TwistStamped current_velocity;
void velo_cb(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    current_velocity = *msg;
}

state_machine::requestCommand_L2F takeOffCommand;
bool receiveTakeOffCommand_enable = true;
void takeOffCommand_cb(const state_machine::requestCommand_L2F::ConstPtr& msg)
{
    if(receiveTakeOffCommand_enable == true)
    {
        takeOffCommand = *msg;
        ROS_INFO("received the take off command!!");
        receiveTakeOffCommand_enable = false;
    } 
}

state_machine::requestCommand_L2F landCommand;
void landCommand_cb(const state_machine::requestCommand_L2F::ConstPtr& msg)
{
    landCommand = *msg;
}

state_machine::requestCommand_L2F emergencyCommand;
bool receiveEmergencyCommand_enable = true;
void emergencyCommand_cb(const state_machine::requestCommand_L2F::ConstPtr& msg)
{
    if(receiveEmergencyCommand_enable == true)
    {
        emergencyCommand = *msg;
        ROS_INFO("received the emergency command!!");
        receiveEmergencyCommand_enable = false;
    } 
}

state_machine::requestCommand_L2F communication_command;
void communication_test_cb(const state_machine::requestCommand_L2F::ConstPtr& msg)
{
    communication_command = *msg;
    communication_result_pub.publish(communicationStatus);
    ROS_INFO("receive the communication request.");
}

state_machine::positionXY position_delta;
void position_delta_cb(const state_machine::positionXY::ConstPtr& msg)
{
    position_delta = *msg; 
    position_next.pose.position.x =  position_A.pose.position.x + position_delta.x;
    position_next.pose.position.y =  position_A.pose.position.y + position_delta.y;
    position_next.pose.position.z =  position_A.pose.position.z;
}

state_machine::Attitude current_attitude;
void attitude_cb(const state_machine::Attitude::ConstPtr& msg)
{
    current_attitude = *msg;
    current_attitude_pub.publish(current_attitude);

    // ROS_INFO("yaw: %f", current_attitude.yaw);
    // ROS_INFO("pitch: %f", current_attitude.pitch);
    // ROS_INFO("roll: %f", current_attitude.roll);
}

state_machine::GPS_Status gps_status;
void gps_status_cb(const state_machine::GPS_Status::ConstPtr& msg)
{
    gps_status = *msg; //0-1: no fix, 2: 2D fix, 3: 3D fix, 4: DGPS, 5: RTK.
    current_gps_status_pub.publish(gps_status); 
    // ROS_INFO("fix_type: %d", gps_status.fix_type);
    // ROS_INFO("satellites_num: %d", gps_status.satellites_num);
}

sensor_msgs::BatteryState battery_state;
void battery_cb(const sensor_msgs::BatteryState::ConstPtr& msg)
{
    battery_state = *msg;
    battery_staus_pub.publish(battery_state);
    // ROS_INFO("voltage: %f", battery_state.voltage);
    // ROS_INFO("current: %f", battery_state.current);
    // ROS_INFO("remaining: %f", battery_state.percentage);
}

/*****************************main function*****************************/
int main(int argc, char **argv)
{
    ros::init(argc, argv, "collborate_uav_3");
    ros::NodeHandle nh;

    takeOffStatus.UAV_index = 2;
    takeOffStatus.value = 1;

    communicationStatus.UAV_index = 2;
    communicationStatus.value = 1;
    
    emergencyStatus.UAV_index = 2;
    emergencyStatus.value = 1;

    //takeoff velocity
    vel_pub.twist.linear.x = 0.0f;
    vel_pub.twist.linear.y = 0.0f;
    vel_pub.twist.linear.z = ASCEND_VELOCITY;
    vel_pub.twist.angular.x = 0.0f;
    vel_pub.twist.angular.y = 0.0f;
    vel_pub.twist.angular.z = 0.0f;
    //position of A
    position_A.pose.position.x = 0;
    position_A.pose.position.y = 0;
    position_A.pose.position.z = 5;
    //position of B
    position_B.pose.position.x = 0;
    position_B.pose.position.y = 5;
    position_B.pose.position.z = 5;
    //position of C
    position_C.pose.position.x = 5;
    position_C.pose.position.y = 5;
    position_C.pose.position.z = 5;
    
    //adjust angular,face north  //just face east is OK
    float yaw_sp=0;
    //A
    position_A.pose.orientation.x = 0;
    position_A.pose.orientation.y = 0;
    position_A.pose.orientation.z = sin(yaw_sp/2);
    position_A.pose.orientation.w = cos(yaw_sp/2);
    //B
    position_B.pose.orientation.x = 0;
    position_B.pose.orientation.y = 0;
    position_B.pose.orientation.z = sin(yaw_sp/2);
    position_B.pose.orientation.w = cos(yaw_sp/2);
    //C
    position_C.pose.orientation.x = 0;
    position_C.pose.orientation.y = 0;
    position_C.pose.orientation.z = sin(yaw_sp/2);
    position_C.pose.orientation.w = cos(yaw_sp/2);

    ros::Subscriber state_sub = nh.subscribe<state_machine::State>("uav3/mavros/state",10,state_cb);
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("uav3/mavros/local_position/pose",10,pose_cb);
    ros::Subscriber vel_sub = nh.subscribe<geometry_msgs::TwistStamped>("uav3/mavros/local_position/velocity",10,velo_cb);
    ros::Subscriber takeOffCommand_sub = nh.subscribe<state_machine::requestCommand_L2F>("take_off_command",10,takeOffCommand_cb);
    ros::Subscriber landCommand_sub = nh.subscribe<state_machine::requestCommand_L2F>("land_command",10,landCommand_cb);
    ros::Subscriber communication_test_sub = nh.subscribe<state_machine::requestCommand_L2F>("communication_test",10,communication_test_cb);
    ros::Subscriber emergency_sub = nh.subscribe<state_machine::requestCommand_L2F>("emergency_command",10,emergencyCommand_cb);
    ros::Subscriber position_delta_sub = nh.subscribe<state_machine::positionXY>("position_delta",10,position_delta_cb);
    //added for gs
    ros::Subscriber attitude_sub = nh.subscribe<state_machine::Attitude>("uav3/mavros/attitude",10,attitude_cb);
    ros::Subscriber gps_staus_sub = nh.subscribe<state_machine::GPS_Status>("uav3/mavros/gps_status",10,gps_status_cb);
    ros::Subscriber battery_staus_sub = nh.subscribe<sensor_msgs::BatteryState>("uav3/mavros/battery",10,battery_cb);

    local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("uav3/mavros/setpoint_position/local",10);
    local_vel_pub = nh.advertise<geometry_msgs::TwistStamped>("uav3/mavros/setpoint_velocity/cmd_vel",10);
    takeOffStatus_pub = nh.advertise<state_machine::attributeStatus_F2L>("uav3_take_off_status",10);
    communication_result_pub = nh.advertise<state_machine::attributeStatus_F2L>("uav3_communication_test_reult",10);
    emergency_status_pub = nh.advertise<state_machine::attributeStatus_F2L>("uav3_emergency_status",10);
    current_position_pub = nh.advertise<std_msgs::String>("uav3_current_position_state",10);

    //added
    current_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("uav3_local_pose",10);
    current_attitude_pub = nh.advertise<state_machine::Attitude>("uav3_local_attitude",10);
    battery_staus_pub = nh.advertise<sensor_msgs::BatteryState>("uav3_battery_status",10);
    current_mode_pub = nh.advertise<std_msgs::String>("uav3_current_mode",10);
    current_gps_status_pub = nh.advertise<state_machine::GPS_Status>("uav3_current_gps_status",10);
    task_status_pub = nh.advertise<state_machine::taskStatusMonitor>("uav3_current_mission_state",10);
 
    land_client = nh.serviceClient<state_machine::CommandTOL>("uav3/mavros/cmd/land");
    state_machine::CommandTOL landing_cmd;
    landing_cmd.request.min_pitch = 1.0;
    landing_last_request = ros::Time::now();

    set_mode_client_offboard = nh.serviceClient<state_machine::SetMode>("uav3/mavros/set_mode");
    state_machine::SetMode offb_set_mode;
	offb_set_mode.request.custom_mode = "OFFBOARD";
    last_request = ros::Time::now();

    set_mode_client_posctl = nh.serviceClient<state_machine::SetMode>("uav3/mavros/set_mode");
    state_machine::SetMode posc_set_mode;
	posc_set_mode.request.custom_mode = "POSCTL";
    last_request = ros::Time::now();

    arming_client = nh.serviceClient<state_machine::CommandBool>("uav3/mavros/cmd/arming");
	state_machine::CommandBool arm_cmd;
	arm_cmd.request.value = true;
    landing_last_request = ros::Time::now();

    ros::Rate rate(20.0);

    while(ros::ok() && !current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("Connect successfully!!");

    //firstly switch to position control mode before switch to offboard mode
    while(ros::ok() && current_state.connected && current_state.mode != "POSCTL")
    {
        if(ros::Time::now() - last_request > ros::Duration(5.0))
        {
            if(set_mode_client_posctl.call(posc_set_mode) && posc_set_mode.response.success) //old version was success, new version is mode_sent
            {
                ROS_INFO("position control enabled");
            }
            last_request = ros::Time::now();
        }
        
        ros::spinOnce();
        rate.sleep();
    }

    while(ros::ok() && current_state.connected)
    {
        static bool display_flag = true;

        if(current_state.mode == "OFFBOARD" && current_state.armed)
        {
            current_position_pub.publish(current_position_state);
            state_machine_fun();
            ROS_INFO("current_pos_state:%d",current_pos_state);
            ROS_INFO("current_mode:%s",current_state.mode.c_str());
        }
        else if(velocity_control_enable == true)
        {
            local_vel_pub.publish(vel_pub);
            if(display_flag == true)
            {
                ROS_INFO("Wait for switch to offboard...");
                display_flag = false;
            }
        }

        if(takeOffCommand.value == 1 && !current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0)))
        {
            if( arming_client.call(arm_cmd) && arm_cmd.response.success)
            {
                ROS_INFO("Vehicle armed");
                get_home_position_enable = true;
                while(home_position_gotten == false)
                {
                    ros::spinOnce();
                    rate.sleep();
                }
            }
            last_request = ros::Time::now();
        }

        if(takeOffCommand.value && current_state.armed && current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0)))
        {
            if( set_mode_client_offboard.call(offb_set_mode) && offb_set_mode.response.success) //old version was success, new version is mode_sent
            {
                ROS_INFO("Offboard enabled");
                takeOffCommand.value = 0;
                takeOffStatus_pub.publish(takeOffStatus);
            }
            last_request = ros::Time::now();
        }
        
        if(current_state.armed && current_pos_state == land)
        {
            if(current_state.mode != "MANUAL" && current_state.mode != "AUTO.LAND" && 
              (ros::Time::now() - landing_last_request > ros::Duration(5.0)))
                {
                    if(land_client.call(landing_cmd) && landing_cmd.response.success)
                    {
                        ROS_INFO("AUTO LANDING"); 
                    }
                    landing_last_request = ros::Time::now();
                }
        }

        if(emergencyCommand.value)
        {
            current_pos_state = current_position_hover;
            hover_position = current_position;
            emergency_status_pub.publish(emergencyStatus);
            emergencyCommand.value = 0;
        }

        //send mission state
        task_status_monitor.task_status = current_position_state.data;
        task_status_monitor.target_x = pose_pub.pose.position.y;
        task_status_monitor.target_y = pose_pub.pose.position.x;
        task_status_monitor.target_z = pose_pub.pose.position.z;
        task_status_pub.publish(task_status_monitor);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

/***********************************state_machine function***********************************/
void state_machine_fun(void)
{
    static bool return_flag = false;

    switch(current_pos_state)
    {
        case takeoff:
        {
            velocity_control_enable = false;
            pose_pub = position_A;
            local_vel_pub.publish(vel_pub);
            if(current_position.pose.position.z > take_off_height)
            {
                current_pos_state = position_A_go;
                last_time = ros::Time::now();
            }
        }
        break;
        case position_A_go:
        {
            pose_pub = position_A;
            local_pos_pub.publish(position_A);
            if (Distance_of_Two(current_position.pose.position.x,position_A.pose.position.x,
                                current_position.pose.position.y,position_A.pose.position.y,
                                current_position.pose.position.z,position_A.pose.position.z) < LOCATE_ACCURACY)
            {
                current_pos_state = position_tracking;
                last_time = ros::Time::now();
            }
        }
        break;
        case position_tracking:
        {
            pose_pub = position_next;
            local_pos_pub.publish(position_next);

            if (Distance_of_Two(current_position.pose.position.x,position_A.pose.position.x,
                                current_position.pose.position.y,position_A.pose.position.y,
                                current_position.pose.position.z,position_A.pose.position.z) < LOCATE_ACCURACY && !return_flag)
            {
                current_position_state.data = "position_A_hover";
            }

            if (Distance_of_Two(current_position.pose.position.x,position_B.pose.position.x,
                                current_position.pose.position.y,position_B.pose.position.y,
                                current_position.pose.position.z,position_B.pose.position.z) < LOCATE_ACCURACY)
            {
                current_position_state.data = "position_B_hover";
            }

            if (Distance_of_Two(current_position.pose.position.x,position_C.pose.position.x,
                                current_position.pose.position.y,position_C.pose.position.y,
                                current_position.pose.position.z,position_C.pose.position.z) < LOCATE_ACCURACY)
            {
                current_position_state.data = "position_C_hover";
                return_flag = true;
            }

            if (Distance_of_Two(current_position.pose.position.x,position_A.pose.position.x,
                                current_position.pose.position.y,position_A.pose.position.y,
                                current_position.pose.position.z,position_A.pose.position.z) < LOCATE_ACCURACY && return_flag)
            {
                current_position_state.data = "position_end_hover";  
            }

            if(landCommand.value)
            {
                current_pos_state = land;
                last_time = ros::Time::now();
            }
        }
        break;
        case current_position_hover:
        {
            pose_pub = hover_position;
            local_pos_pub.publish(hover_position);
            if(ros::Time::now() - last_time > ros::Duration(5.0))
            {
                current_pos_state = land;
                last_time = ros::Time::now();
            }
        }
        break;
        case land:
        break;
    }
}

/**************************************function definition**************************************/

double Distance_of_Two(double x1, double x2, double y1, double y2, double z1, double z2)
{
    return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1)+(z2-z1)*(z2-z1));
}