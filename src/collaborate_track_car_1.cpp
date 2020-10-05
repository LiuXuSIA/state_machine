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
#include <state_machine/VisionCarBoardPos.h>
#include <state_machine/HeadBoardYaw.h>
#include "math.h"
#include "string"
#include "vector"

using namespace std;

/***************************function declare****************************/

void state_machine_fun(void);
double Distance_of_Two(double x1, double x2, double y1, double y2, double z1, double z2);
float wrap_pi(float angle_rad);

/***************************variable definition*************************/
//position  ENU
geometry_msgs::PoseStamped position_A;
geometry_msgs::PoseStamped hover_position;
float yaw_sp;

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
ros::Publisher car_yaw_pub;


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
// ros::Subscriber car_board_position_sub;
// ros::Subscriber car_head_position_sub;
// ros::Subscriber car_position_sub;
// ros::Subscriber car_num_sub;
ros::Subscriber vision_information_sub;

//message to gs
std_msgs::Int8 takeOffStatus;
std_msgs::Int8 hoverStatus;
std_msgs::Int8 landStatus;
std_msgs::Int8 communicationStatus;
std_msgs::Int8 remoteControlStatus;
std_msgs::Int8 tracked_car;
geometry_msgs::PoseStamped car_position;
std_msgs::Int8 car_number;

//exchange maxtric
float R[3][3] = {0};

//state_machine mission state
//every state need target position

static const int wait_TF = 0;
static const int takeoff = 1;
static const int position_A_go = 2;
static const int position_A_hover = 3;
static const int car_tracking = 4;
static const int return_home = 5;
static const int land = 6;

static const int current_position_hover = 10;
static const int land_after_hover = 11;

static string mission_status[12] = {"wait_TF", "take_off", "go_A", "hover_A", "go_B", "hover_B", "go_C", "hover_C", 
                                    "return_H", "land", "hover", "L_hover"};

//mission 
int loop = 0;
int current_mission_state = wait_TF;
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

#define NS                  "uav1"
#define NS_CAR              "car1"
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

        take_off_height = position_A.pose.position.z-2;

        ROS_INFO("gotten the home position.");
        ROS_INFO("the x of home:%f", position_A.pose.position.x);
        ROS_INFO("the y of home:%f", position_A.pose.position.y);
        ROS_INFO("the z of home:%f", position_A.pose.position.z);

        get_home_position_enable = false;
        home_position_gotten = true;
    }
}

state_machine::Attitude current_attitude;
void attitude_cb(const state_machine::Attitude::ConstPtr& msg)
{
    current_attitude = *msg;
    current_attitude_pub.publish(current_attitude);

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

    // ROS_INFO("yaw: %f", current_attitude.yaw);
    // ROS_INFO("pitch: %f", current_attitude.pitch);
    // ROS_INFO("roll: %f", current_attitude.roll);
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
    current_mission_state = takeoff;
    sm_control_mode = true;
    velocity_control_enable = true;
    takeOffStatus.data = 1;
    takeOffStatus_pub.publish(takeOffStatus);
    ROS_INFO("received the take off command!!");
}

std_msgs::Int8 landCommand;
void landCommand_cb(const std_msgs::Int8::ConstPtr& msg)
{
    landCommand = *msg;
    current_mission_state = land_after_hover;
    hover_position = current_position;
    landStatus.data = 1;
    land_status_pub.publish(landStatus);
    ROS_INFO("received the land command!!");
}

std_msgs::Int8 hoverCommand;
void hoverCommand_cb(const std_msgs::Int8::ConstPtr& msg)
{
    hoverCommand = *msg;
    current_mission_state = current_position_hover;
    hover_position = current_position;
    hoverStatus.data = 1;
    hover_status_pub.publish(hoverStatus);
    ROS_INFO("received the hover command!!");
}

std_msgs::Int8 communication_command;
void communication_test_cb(const std_msgs::Int8::ConstPtr& msg)
{
    communication_command = *msg;
    communicationStatus.data = 1;
    communication_result_pub.publish(communicationStatus);
    ROS_INFO("receive the communication request.");
}

std_msgs::Int8 control_request;
geometry_msgs::PoseStamped local_pose_G2M;
void control_request_cb(const std_msgs::Int8::ConstPtr& msg)
{
    control_request = *msg;
    if(control_request.data == 1)
    {
        local_pose_G2M = current_position;
        sm_control_mode = false;
        remoteControlStatus.data = 1;
        remote_control_status_pub.publish(remoteControlStatus);
        ROS_INFO("receive the control request.");
    }
    else if(control_request.data == 0)
    {
        current_mission_state = current_position_hover;
        sm_control_mode = true;
        remoteControlStatus.data = 2;
        remote_control_status_pub.publish(remoteControlStatus);
        ROS_INFO("exit the control request.");
    }
}

void local_position_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    local_pose_G2M = *msg; 
    ROS_INFO("receive the control position x:%f, y:%f, z:%f",
    local_pose_G2M.pose.position.x,local_pose_G2M.pose.position.y,local_pose_G2M.pose.position.z);
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
state_machine::VisionCarBoardPos vision_ms;
state_machine::HeadBoardYaw headBoardYaw_ms;
geometry_msgs::PoseStamped car_pose;
geometry_msgs::PoseStamped tracked_position, last_tracked_position;
geometry_msgs::Point body_car, body_board, body_head, body_tracked, local_position_car, local_position_board, local_position_head, local_position_tracked;
float delta_y, delta_x, yaw_calculated, last_yaw;;
std_msgs::Int8 board_num;
int head_lost, board_lost, car_lost;
void car_information_cb(const state_machine::VisionCarBoardPos::ConstPtr& msg)
{
    vision_ms = *msg;

    ROS_INFO("body_car:x%f y%f z%f", vision_ms.car.x, vision_ms.car.y, vision_ms.car.z);
    ROS_INFO("body_board:x%f y%f z%f", vision_ms.board.x, vision_ms.board.y, vision_ms.board.z);
    ROS_INFO("body_head:x%f y%f z%f", vision_ms.head.x, vision_ms.head.y, vision_ms.head.z);
    ROS_INFO("board_num:%d", vision_ms.boardNum);

    if(vision_ms.car.z == 0)
    {
        car_lost++;
    }
    else
    {
        body_car.x = vision_ms.car.z;
        body_car.y = vision_ms.car.x;
        body_car.z = vision_ms.car.y;
    }
    if(vision_ms.board.z == 0)
    {
        board_lost++;
    }
    else
    {
        body_board.x = vision_ms.board.z;
        body_board.y = vision_ms.board.x;
        body_board.z = vision_ms.board.y;
        board_num.data = vision_ms.boardNum;
    }
    if(vision_ms.head.z == 0)
    {
        head_lost++;
    }
    else
    {
        body_head.x = vision_ms.head.z;
        body_head.y = vision_ms.head.x;
        body_head.z = vision_ms.head.y;

        body_tracked.x = vision_ms.head.z-5;
        body_tracked.y = vision_ms.head.x;
        body_tracked.z = vision_ms.head.y;
    }
    if(board_lost > 1 || head_lost > 1)
    {
        current_mission_state = land_after_hover;
    }

    local_position_car.x = (R[0][0] * body_car.x + R[0][1] * body_car.y + R[0][2] * body_car.z) + current_position.pose.position.y;
    local_position_car.y = (R[1][0] * body_car.x + R[1][1] * body_car.y + R[1][2] * body_car.z) + current_position.pose.position.x;
    local_position_car.z = (R[2][0] * body_car.x + R[2][1] * body_car.y + R[2][2] * body_car.z) - current_position.pose.position.z;

    local_position_board.x = (R[0][0] * body_board.x + R[0][1] * body_board.y + R[0][2] * body_board.z) + current_position.pose.position.y;
    local_position_board.y = (R[1][0] * body_board.x + R[1][1] * body_board.y + R[1][2] * body_board.z) + current_position.pose.position.x;
    local_position_board.z = (R[2][0] * body_board.x + R[2][1] * body_board.y + R[2][2] * body_board.z) - current_position.pose.position.z;

    local_position_head.x = (R[0][0] * body_head.x + R[0][1] * body_head.y + R[0][2] * body_head.z) + current_position.pose.position.y;
    local_position_head.y = (R[1][0] * body_head.x + R[1][1] * body_head.y + R[1][2] * body_head.z) + current_position.pose.position.x;
    local_position_head.z = (R[2][0] * body_head.x + R[2][1] * body_head.y + R[2][2] * body_head.z) - current_position.pose.position.z;

    local_position_tracked.x = (R[0][0] * body_tracked.x + R[0][1] * body_tracked.y + R[0][2] * body_tracked.z) + current_position.pose.position.y;
    local_position_tracked.y = (R[1][0] * body_tracked.x + R[1][1] * body_tracked.y + R[1][2] * body_tracked.z) + current_position.pose.position.x;
    local_position_tracked.z = (R[2][0] * body_tracked.x + R[2][1] * body_tracked.y + R[2][2] * body_tracked.z) - current_position.pose.position.z;

    delta_y = local_position_head.y - local_position_board.y;
    delta_x = local_position_head.x - local_position_board.x;

    yaw_calculated = atan2(delta_y,delta_x);
    yaw_sp = wrap_pi(M_PI_2 - yaw_calculated * M_PI/180);
    if((yaw_sp - last_yaw) > 1)
    {
       yaw_sp = last_yaw;
    } 
    last_yaw = yaw_sp;

    tracked_position.pose.position.x = local_position_tracked.y;
    tracked_position.pose.position.y = local_position_tracked.x;
    tracked_position.pose.position.z = -local_position_tracked.z;
    tracked_position.pose.orientation.x = 0;
    tracked_position.pose.orientation.y = 0;
    tracked_position.pose.orientation.z = sin(yaw_sp/2);
    tracked_position.pose.orientation.w = cos(yaw_sp/2);

    car_pose.pose.position.x = local_position_car.y;
    car_pose.pose.position.y = local_position_car.x;
    car_pose.pose.position.z = -local_position_car.z;
    headBoardYaw_ms.board.x = local_position_board.y;
    headBoardYaw_ms.board.y = local_position_board.x;
    headBoardYaw_ms.board.z = -local_position_board.z;
    headBoardYaw_ms.head.x = local_position_head.y;
    headBoardYaw_ms.head.y = local_position_head.x;
    headBoardYaw_ms.head.z = -local_position_head.z;
    headBoardYaw_ms.car_yaw = yaw_sp;

    car_position_pub.publish(car_pose);
    car_num_pub.publish(board_num);
    car_yaw_pub.publish(headBoardYaw_ms);
}

// geometry_msgs::PoseStamped car1_head_position;
// void car_head_position_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
// {
//     car1_head_position = *msg;
// }

// geometry_msgs::PoseStamped car1_position;
// void car_position_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
// {
//     car1_position = *msg;
//     car_position_pub.publish(car1_position);
// }

// std_msgs::Int8 car1_num;
// void car_num_cb(const std_msgs::Int8::ConstPtr& msg)
// {
//     car1_num = *msg;
//     car_num_pub.publish(car1_num);
// }
#endif

/*****************************main function*****************************/
int main(int argc, char **argv)
{
    ros::init(argc, argv, "collborate_"+string(NS));
    ros::NodeHandle nh;

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
    
    //A
    position_A.pose.orientation.x = 0;
    position_A.pose.orientation.y = 0;
    position_A.pose.orientation.z = sin(yaw_sp/2);
    position_A.pose.orientation.w = cos(yaw_sp/2);

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
    vision_information_sub = nh.subscribe<state_machine::VisionCarBoardPos>(string(NS_CAR)+"_vision_information",10, car_information_cb);
    // car_board_position_sub = nh.subscribe<geometry_msgs::PoseStamped>(string(NS_CAR)+"_board_vision_position",10,car_board_position_cb);
    // car_head_position_sub = nh.subscribe<geometry_msgs::PoseStamped>(string(NS_CAR)+"_head_vision_position",10,car_head_position_cb);
    // car_position_sub = nh.subscribe<geometry_msgs::PoseStamped>(string(NS_CAR)+"_vision_position",10,car_position_cb);
    // car_num_sub = nh.subscribe<std_msgs::Int8>(string(NS_CAR)+"_num",10,car_num_cb);
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
    car_yaw_pub = nh.advertise<state_machine::HeadBoardYaw>(string(NS_CAR)+"_yaw",10);
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
        
        //send mission state
        task_status_monitor.task_status = mission_status[current_mission_state];
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
                current_mission_state = car_tracking;
                last_time = ros::Time::now();
            }
        }
        break;
        case car_tracking:
        {
            pose_pub = tracked_position;
            local_pos_pub.publish(tracked_position);
        }
        break;
        case return_home:
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
        case current_position_hover:
        {
            pose_pub = hover_position;
            local_pos_pub.publish(hover_position);
            last_time = ros::Time::now();
        }
        break;
        case land_after_hover:
        {
            pose_pub = hover_position;
            local_pos_pub.publish(hover_position);
            if(ros::Time::now() - last_time > ros::Duration(5.0))
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

// limit angle_rad to [-pi,pi]
float wrap_pi(float angle_rad)
{
    if (angle_rad >= M_PI) 
    {
        angle_rad -= M_PI*2;
    }
    if (angle_rad < -M_PI) 
    {
        angle_rad += M_PI*2;
    }
    return angle_rad;
}