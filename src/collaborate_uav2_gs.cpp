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
#include <std_msgs/Int8.h>
#include <state_machine/positionXY.h>
#include <state_machine/SetMode.h>
#include <state_machine/Attitude.h>
#include <state_machine/GPS_Status.h>
#include <sensor_msgs/BatteryState.h>
#include <state_machine/taskStatusMonitor.h>
#include "math.h"
#include "string"
#include "vector"

using namespace std;

/***************************function declare****************************/

void state_machine_fun(void);
double Distance_of_Two(double x1, double x2, double y1, double y2, double z1, double z2);

/***************************variable definition*************************/
//position  ENU
geometry_msgs::PoseStamped position_A;
geometry_msgs::PoseStamped position_B;
geometry_msgs::PoseStamped position_C;
geometry_msgs::PoseStamped hover_position;

//velcity ENU
geometry_msgs::PoseStamped pose_pub;
geometry_msgs::TwistStamped vel_pub;

//published topic
ros::Publisher local_pos_pub;
ros::Publisher local_vel_pub;
ros::Publisher takeOffStatus_pub;
ros::Publisher hover_status_pub;
ros::Publisher land_status_pub;
ros::Publisher communication_result_pub;
ros::Publisher remote_control_status_pub;
ros::Publisher current_pose_pub;
ros::Publisher current_attitude_pub;
ros::Publisher current_gps_status_pub;
ros::Publisher battery_staus_pub;
ros::Publisher current_mode_pub;
ros::Publisher tracked_car_pub;
ros::Publisher task_status_pub;
ros::Publisher car_position_pub;
ros::Publisher car_num_pub;
ros::Publisher current_position_state_pub;


//subscibed topic
ros::Subscriber state_sub;
ros::Subscriber pose_sub;
ros::Subscriber vel_sub;
ros::Subscriber takeOffCommand_sub;
ros::Subscriber landCommand_sub;
ros::Subscriber hoverCommand_sub;
ros::Subscriber controlRequest_sub;
ros::Subscriber communication_test_sub;
ros::Subscriber local_position_G2M_sub;
ros::Subscriber attitude_sub;
ros::Subscriber gps_staus_sub;
ros::Subscriber battery_staus_sub;
ros::Subscriber tracked_car_sub;
ros::Subscriber car_position_sub;
ros::Subscriber car_num_sub;

//message to gs
std_msgs::Int8 takeOffStatus;
std_msgs::Int8 hoverStatus;
std_msgs::Int8 landStatus;
std_msgs::Int8 communicationStatus;
std_msgs::Int8 remoteControlStatus;
std_msgs::Int8 tracked_car;
geometry_msgs::PoseStamped car_position;
std_msgs::Int8 car_number;

//state_machine mission state
//every state need target position

static const int takeoff = 0;
static const int position_A_go = 1;
static const int position_A_hover = 2;
static const int position_B_go = 3;
static const int position_B_hover = 4;
static const int position_C_go = 5;
static const int position_C_hover = 6;
static const int return_home = 7;
static const int land = 8;

static const int current_position_hover = 9;
static const int land_after_hover = 10;

static string mission_status[11] = {"take_off", "go_A", "hover_A", "go_B", "hover_B", "go_C", "hover_C", 
                                    "return_H", "land", "hover", "L_hover"};

//mission 
int loop = 0;
int current_mission_state = takeoff;
state_machine::taskStatusMonitor task_status_monitor;

//time
ros::Time last_time;

//velocity send
bool velocity_control_enable = true;

//takeoff height
double take_off_height;

//control mode
bool sm_control_mode = true;

//get the home position before takeoff
bool get_home_position_enable = false;

//land
ros::ServiceClient land_client;
ros::Time landing_last_request;

ros::ServiceClient arming_client;
ros::ServiceClient set_mode_client_offboard;
ros::ServiceClient set_mode_client_posctl;
ros::Time last_request;

/*************************constant definition***************************/

#define NS                  "uav2"
#define NS_CAR              "car2"
#define FIXED               1
#define ASCEND_VELOCITY     2.0
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

std_msgs::Int8 takeOffCommand;
void takeOffCommand_cb(const std_msgs::Int8::ConstPtr& msg)
{
    takeOffCommand = *msg;
    takeOffStatus_pub.publish(takeOffStatus);
    ROS_INFO("received the take off command!!");
}

std_msgs::Int8 landCommand;
void landCommand_cb(const std_msgs::Int8::ConstPtr& msg)
{
    landCommand = *msg;
    current_mission_state = land_after_hover;
    hover_position = current_position;
    land_status_pub.publish(landStatus);
    ROS_INFO("received the land command!!");
}

std_msgs::Int8 hoverCommand;
void hoverCommand_cb(const std_msgs::Int8::ConstPtr& msg)
{
    hoverCommand = *msg;
    current_mission_state = current_position_hover;
    hover_position = current_position;
    hover_status_pub.publish(hoverStatus);
    ROS_INFO("received the hover command!!");
}

std_msgs::Int8 communication_command;
void communication_test_cb(const std_msgs::Int8::ConstPtr& msg)
{
    communication_command = *msg;
    communication_result_pub.publish(communicationStatus);
    ROS_INFO("receive the communication request.");
}

std_msgs::Int8 control_request;
geometry_msgs::PoseStamped local_pose_G2M;
void control_request_cb(const std_msgs::Int8::ConstPtr& msg)
{
    control_request = *msg;
    local_pose_G2M = current_position;
    sm_control_mode = false;
    remote_control_status_pub.publish(remoteControlStatus);
    ROS_INFO("receive the control request.");
}

void local_position_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    local_pose_G2M = *msg; 
    ROS_INFO("receive the control position x:%f, y:%f, z:%f",
    local_pose_G2M.pose.position.x,local_pose_G2M.pose.position.y,local_pose_G2M.pose.position.z);
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

#if FIXED
geometry_msgs::PoseStamped car1_position;
void car_position_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    car1_position = *msg;
    car_position_pub.publish(car1_position);
}

std_msgs::Int8 car1_num;
void car_num_cb(const std_msgs::Int8::ConstPtr& msg)
{
    car1_num = *msg;
    car_num_pub.publish(car1_num);
}
#endif

/*****************************main function*****************************/
int main(int argc, char **argv)
{
    ros::init(argc, argv, "collborate_"+string(NS));
    ros::NodeHandle nh;

    takeOffStatus.data = 1;
    hoverStatus.data = 1;
    landStatus.data = 1;
    communicationStatus.data = 1;
    remoteControlStatus.data = 1;

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
    
    //subscribe from pix
    state_sub = nh.subscribe<state_machine::State>(string(NS)+"/mavros/state",10,state_cb);
    pose_sub = nh.subscribe<geometry_msgs::PoseStamped>(string(NS)+"/mavros/local_position/pose",10,pose_cb);
    vel_sub = nh.subscribe<geometry_msgs::TwistStamped>(string(NS)+"/mavros/local_position/velocity",10,velo_cb);
    attitude_sub = nh.subscribe<state_machine::Attitude>(string(NS)+"/mavros/attitude",10,attitude_cb);
    gps_staus_sub = nh.subscribe<state_machine::GPS_Status>(string(NS)+"/mavros/gps_status",10,gps_status_cb);
    battery_staus_sub = nh.subscribe<sensor_msgs::BatteryState>(string(NS)+"/mavros/battery",10,battery_cb);
    //subscribe from GS
    communication_test_sub = nh.subscribe<std_msgs::Int8>("communication_test",10,communication_test_cb);
    takeOffCommand_sub = nh.subscribe<std_msgs::Int8>("take_off_command",10,takeOffCommand_cb);
    landCommand_sub = nh.subscribe<std_msgs::Int8>("land_command",10,landCommand_cb);
    hoverCommand_sub = nh.subscribe<std_msgs::Int8>("hover_command",10,hoverCommand_cb);
    controlRequest_sub = nh.subscribe<std_msgs::Int8>("control_request",10,control_request_cb);
    local_position_G2M_sub = nh.subscribe<geometry_msgs::PoseStamped>(string(NS)+"_local_position",10,local_position_cb);
    //subscribe from vision
    #if FIXED
    //tracked_car_sub = nh.advertise<std_msgs::Int8>(string(NS)+"_tracked_car",10);
    car_position_sub = nh.subscribe<geometry_msgs::PoseStamped>(string(NS_CAR)+"_pos",10,car_position_cb);
    car_num_sub = nh.subscribe<std_msgs::Int8>(string(NS_CAR)+"_num",10,car_num_cb);
    #endif
    
    //publish to pix
    local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>(string(NS)+"/mavros/setpoint_position/local",10);
    local_vel_pub = nh.advertise<geometry_msgs::TwistStamped>(string(NS)+"/mavros/setpoint_velocity/cmd_vel",10);
    //publish to GS
    takeOffStatus_pub = nh.advertise<std_msgs::Int8>(string(NS)+"_take_off_receive",10);
    hover_status_pub = nh.advertise<std_msgs::Int8>(string(NS)+"_hover_receive",10);
    land_status_pub = nh.advertise<std_msgs::Int8>(string(NS)+"_land_receive",10);
    communication_result_pub = nh.advertise<std_msgs::Int8>(string(NS)+"_communication_test_status",10);
    remote_control_status_pub = nh.advertise<std_msgs::Int8>(string(NS)+"_remote_control_flag",10);
    current_pose_pub = nh.advertise<geometry_msgs::PoseStamped>(string(NS)+"_local_pose",10);
    current_attitude_pub = nh.advertise<state_machine::Attitude>(string(NS)+"_local_attitude",10);
    battery_staus_pub = nh.advertise<sensor_msgs::BatteryState>(string(NS)+"_battery_status",10);
    current_mode_pub = nh.advertise<std_msgs::String>(string(NS)+"_current_mode",10);
    current_gps_status_pub = nh.advertise<state_machine::GPS_Status>(string(NS)+"_current_gps_status",10);
    task_status_pub = nh.advertise<state_machine::taskStatusMonitor>(string(NS)+"_current_mission_state",10);
    #if FIXED
    //tracked_car_pub = nh.advertise<std_msgs::Int8>(string(NS)+"_tracked_car",10);
    car_position_pub = nh.advertise<geometry_msgs::PoseStamped>(string(NS_CAR)+"_position",10);
    car_num_pub = nh.advertise<std_msgs::Int8>(string(NS_CAR)+"_number",10);
    #endif
    
    //service client:land client
    land_client = nh.serviceClient<state_machine::CommandTOL>(string(NS)+"/mavros/cmd/land");
    state_machine::CommandTOL landing_cmd;
    landing_cmd.request.min_pitch = 1.0;
    landing_last_request = ros::Time::now();

    //service client:switch mode to offboard
    set_mode_client_offboard = nh.serviceClient<state_machine::SetMode>(string(NS)+"/mavros/set_mode");
    state_machine::SetMode offb_set_mode;
	offb_set_mode.request.custom_mode = "OFFBOARD";
    last_request = ros::Time::now();
    
    //service client:switch mode to position control
    set_mode_client_posctl = nh.serviceClient<state_machine::SetMode>(string(NS)+"/mavros/set_mode");
    state_machine::SetMode posc_set_mode;
	posc_set_mode.request.custom_mode = "POSCTL";
    last_request = ros::Time::now();
    
    //service client:unlock the uav
    arming_client = nh.serviceClient<state_machine::CommandBool>(string(NS)+"/mavros/cmd/arming");
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
            if(sm_control_mode == true)
            {
                state_machine_fun();

                ROS_INFO("current_mission_state:%d",current_mission_state);
                ROS_INFO("current_mission_state:%s",mission_status[current_mission_state].c_str());
                ROS_INFO("current_mode:%s",current_state.mode.c_str());

                task_status_monitor.task_status = mission_status[current_mission_state];
                task_status_monitor.target_x = pose_pub.pose.position.y;
                task_status_monitor.target_y = pose_pub.pose.position.x;
                task_status_monitor.target_z = pose_pub.pose.position.z;
                task_status_pub.publish(task_status_monitor);
            }
            else
            {
                local_pos_pub.publish(local_pose_G2M);
            }
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

        if(takeOffCommand.data && !current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0)))
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

        if(takeOffCommand.data && current_state.armed && current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0)))
        {
            if( set_mode_client_offboard.call(offb_set_mode) && offb_set_mode.response.success) //old version was success, new version is mode_sent
            {
                ROS_INFO("Offboard enabled");
                takeOffCommand.data = 0;
            }
            last_request = ros::Time::now();
        }
        
        if(current_state.armed && current_mission_state == land)
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

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

/***********************************state_machine function***********************************/
void state_machine_fun(void)
{
    switch(current_mission_state)
    {
        case takeoff:
        {
            velocity_control_enable = false;
            pose_pub = position_A;
            local_vel_pub.publish(vel_pub);
            if(current_position.pose.position.z > take_off_height)
            {
                current_mission_state = position_A_go;
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
                current_mission_state = position_A_hover;
                last_time = ros::Time::now();
            }
        }
        break;
        case position_A_hover:
        {
            pose_pub = position_A;
            local_pos_pub.publish(position_A);
            if(ros::Time::now() - last_time > ros::Duration(5.0))
            {
                current_mission_state = position_B_go;
                last_time = ros::Time::now();
            }
        }
        break;
        case position_B_go:
        {
            pose_pub = position_B;
            local_pos_pub.publish(position_B);
            if (Distance_of_Two(current_position.pose.position.x,position_B.pose.position.x,
                                current_position.pose.position.y,position_B.pose.position.y,
                                current_position.pose.position.z,position_B.pose.position.z) < LOCATE_ACCURACY)
            {
                current_mission_state = position_B_hover;
                last_time = ros::Time::now();
            }
        }
        break;
        case position_B_hover:
        {
            pose_pub = position_B;
            local_pos_pub.publish(position_B);
            if(ros::Time::now() - last_time > ros::Duration(5.0))
            {
                current_mission_state = position_C_go;
                last_time = ros::Time::now();
            }
        }
        break;
        case position_C_go:
        {
            pose_pub = position_C;
            local_pos_pub.publish(position_C);
            if (Distance_of_Two(current_position.pose.position.x,position_C.pose.position.x,
                                current_position.pose.position.y,position_C.pose.position.y,
                                current_position.pose.position.z,position_C.pose.position.z) < LOCATE_ACCURACY)
            {
                current_mission_state = position_C_hover;
                last_time = ros::Time::now();
            }
        }
        break;
        case position_C_hover:
        {
            pose_pub = position_C;
            local_pos_pub.publish(position_C);
            if(ros::Time::now() - last_time > ros::Duration(5.0))
            {
                current_mission_state = return_home;
                last_time = ros::Time::now();
            }
        }
        break;
        case  return_home:
        {
            pose_pub = position_A;
            local_pos_pub.publish(position_A);
            if (Distance_of_Two(current_position.pose.position.x,position_A.pose.position.x,
                                current_position.pose.position.y,position_A.pose.position.y,
                                current_position.pose.position.z,position_A.pose.position.z) < LOCATE_ACCURACY )
            {
                current_mission_state = land;
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