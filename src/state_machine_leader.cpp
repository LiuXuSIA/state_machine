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
#include <state_machine/State.h>
#include <state_machine/takeOffCommand_L2F.h>
#include <state_machine/takeOffStatus_F2L.h>
#include "math.h"


/***************************function declare****************************/

void state_machine_fun(void);
double Distance_of_Two(double x1, double x2, double y1, double y2, double z1, double z2);

/***************************variable definition*************************/
//position  ENU
geometry_msgs::PoseStamped position_A;
geometry_msgs::PoseStamped position_B;
geometry_msgs::PoseStamped position_C;

geometry_msgs::PoseStamped pose_pub;  //ENU
geometry_msgs::TwistStamped vel_pub;

//for take off
ros::Publisher local_pos_pub;
ros::Publisher local_vel_pub;
ros::Publisher takeOffCommand_pub;

//to other uavs
state_machine::takeOffCommand_L2F takeOffCommand;

//bool velocity_control_enable = true;

//state_machine state
//every state need target position
static const int takeoff = 1;
static const int position_A_go = 2;
static const int position_A_hover = 3;
static const int position_B_go = 4;
static const int position_B_hover = 5;
static const int position_C_go = 6;
static const int position_C_hover = 7;
static const int return_home = 8;
static const int land = 9;

//mission 
int loop = 0;
int current_pos_state = takeoff;

//time
ros::Time last_time;

//velocity send
bool velocity_control_enable = true;

//land
state_machine::CommandTOL landing_cmd;
ros::ServiceClient land_client;
ros::Time landing_last_request;

/*************************constant defunition***************************/

#define ASCEND_VELOCITY     2.0
#define LOCATE_ACCURACY     0.5

/*************************simulation test******************************/
#define SIMULATION 1


/***************************callback function definition***************/
state_machine::State current_state;
void state_cb(const state_machine::State::ConstPtr& msg)
{
    current_state = *msg;
}

geometry_msgs::PoseStamped current_position;
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    current_position = *msg;
}

geometry_msgs::TwistStamped current_velocity;
void velo_cb(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    current_velocity = *msg;
}

state_machine::takeOffStatus_F2L takeoffStatus_uav1;
void takeOff_uav1_cb(const state_machine::takeOffStatus_F2L::ConstPtr& msg)
{
    takeoffStatus_uav1 = *msg;
    ROS_INFO("uav1 switched to offboard successfully!!");
}

/*****************************main function*****************************/
int main(int argc, char **argv)
{
    ros::init(argc, argv, "offboard_node_leader");
    ros::NodeHandle nh;
    
    takeOffCommand.value = 1;
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

    ros::Subscriber state_sub = nh.subscribe<state_machine::State>("mavros/state",10,state_cb);
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose",10,pose_cb);
    ros::Subscriber vel_sub = nh.subscribe<geometry_msgs::TwistStamped>("mavros/local_position/velocity",10,velo_cb);
    ros::Subscriber takeOffStatus_uav1_sub = nh.subscribe<state_machine::takeOffStatus_F2L>("takeOffStatus_uav1",10,takeOff_uav1_cb);
    // ros::Subscriber takeOffStatus_uav2_sub = nh.subscribe<state_machine::takeOffStatus_F2L>("takeOffStatus_uav2",10,takeOff_uav2_cb);
    // ros::Subscriber takeOffStatus_uav3_sub = nh.subscribe<state_machine::takeOffStatus_F2L>("takeOffStatus_uav3",10,takeOff_uav3_cb);
    // ros::Subscriber takeOffStatus_uav4_sub = nh.subscribe<state_machine::takeOffStatus_F2L>("takeOffStatus_uav4",10,takeOff_uav4_cb);

    local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local",10);
    local_vel_pub = nh.advertise<geometry_msgs::TwistStamped>("mavros/setpoint_velocity/cmd_vel",10);

    takeOffCommand_pub = nh.advertise<state_machine::takeOffCommand_L2F>("takeOffCommand",10);

    land_client = nh.serviceClient<state_machine::CommandTOL>("mavros/cmd/land");
    landing_cmd.request.min_pitch = 1.0;
    landing_last_request = ros::Time::now();

    ros::Rate rate(10.0);

    #if SIMULATION == 0
    while(ros::ok() && !current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("Connect successfully!!");
    #endif

    ROS_INFO("send setpoint before takeoff,please wait");

    for(int i =100; ros::ok() && i > 0; i--)
    {
        local_vel_pub.publish(vel_pub);
        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("Initialization finished");

    while(ros::ok())
    {
        static bool display_flag = true;

        #if SIMULATION
        if(takeoffStatus_uav1.value != 1)
        {
            takeOffCommand_pub.publish(takeOffCommand);
            ROS_INFO("send the take off command!!");
        }
        #else
        if(current_state.mode == "OFFBOARD" && current_state.armed)
        {
            state_machine_fun();

            if(takeoffStatus_uav1.value != 1)
            {
                takeOffCommand_pub.publish(takeOffCommand);
            }
            else
            {
                ROS_INFO("uav1 has been switched to offboard!!");
            }
            ROS_INFO("current_pos_state:%d",current_pos_state);
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
        #endif

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

/***********************************state_machine function***********************************/
void state_machine_fun(void)
{
    switch(current_pos_state)
    {
        case takeoff:
        {
            velocity_control_enable = false;
            pose_pub = position_A;
            local_vel_pub.publish(vel_pub);
            if(current_position.pose.position.z > 4)
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
                current_pos_state = position_A_hover;
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
                current_pos_state = position_B_go;
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
                current_pos_state = position_B_hover;
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
                current_pos_state = position_C_go;
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
                current_pos_state = position_C_hover;
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
                current_pos_state = return_home;
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