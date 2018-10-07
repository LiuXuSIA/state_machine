/*************************************************************************
@file           state_machine_v1.2.cpp
@date           2018/08/20 16:25
@author         liuxu
@email          liuxu.ccc@gmail.com
@description    state machine for the drone race in 2018
*************************************************************************/

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <state_machine/CommandBool.h>
#include <state_machine/CommandTOL.h>
#include <state_machine/SetMode.h>
#include <state_machine/State.h>
#include "math.h"

//custom gcs->pix->mavros
#include <state_machine/FIXED_TARGET_POSITION_P2M.h>
#include <state_machine/TASK_STATUS_CHANGE_P2M.h>

//custom mavros->pix->gcs
#include <state_machine/TASK_STATUS_MONITOR_M2P.h>
#include <state_machine/VISION_POSITION_GET_M2P.h>
#include <state_machine/YAW_SP_CALCULATED_M2P.h>
#include <state_machine/FIXED_TARGET_RETURN_M2P.h>
#include <state_machine/GRAB_STATUS_M2P.h>

#include <state_machine/Distance.h> 
#include <state_machine/Vision_Position_Raw.h> 


/***************************function declare****************************/

void state_machine_fun(void);
double Distance_of_Two(double x1, double x2, double y1, double y2, double z1, double z2);
float wrap_pi(float angle_rad);

/***************************CONSTANT definition*************************/

#define MAX_MISSION_TIME    500
#define TAKEOFF_HEIGHT      3.0
#define TAKEOFF_VELOCITY    2.0
#define OBSERVE_HEIGET      3.0
#define CONSTRUCT_HEIGET    3.0
#define BOX_HEIGET          0.25
#define GRAB_HEIGET         0.5
#define PLACE_HEIGET        0.5
#define DESCEND_VELOCITY    0.5

/***************************variable definition*************************/
//fixed position  ENU
geometry_msgs::PoseStamped position_home;
geometry_msgs::PoseStamped position_component;
geometry_msgs::PoseStamped position_construction;

//position for hover  ENU
geometry_msgs::PoseStamped position_to_grab;
geometry_msgs::PoseStamped position_to_place;
geometry_msgs::PoseStamped position_to_judge;
geometry_msgs::PoseStamped position_to_component;
geometry_msgs::PoseStamped position_by_calculated;
geometry_msgs::PoseStamped position_return_home;
geometry_msgs::PoseStamped position_after_lose;

//topic
geometry_msgs::PoseStamped pose_target; 
geometry_msgs::TwistStamped vel_ascend;
geometry_msgs::TwistStamped vel_descend;

ros::Publisher local_pos_pub;
ros::Publisher local_vel_pub;
ros::Publisher fixed_target_pub;
ros::Publisher vision_position_pub;

//message to pix
state_machine::FIXED_TARGET_RETURN_M2P fix_target_return;
state_machine::GRAB_STATUS_M2P grab_status;
state_machine::TASK_STATUS_MONITOR_M2P task_status_monitor;
state_machine::VISION_POSITION_GET_M2P vision_position_get;
state_machine::YAW_SP_CALCULATED_M2P yaw_sp_calculated;

//state_machine state,every state need target position
static const int takeoff = 1;
static const int position_home_go = 2;
static const int position_home_hover = 3;
static const int hover_only = 4;
static const int position_component_go = 5;
static const int hover_to_recognize = 6;
static const int component_locate = 7;
static const int hover_to_stable_component = 8;
static const int component_get_close = 9;
static const int component_grab = 10;
static const int component_leave = 11;
static const int grab_status_judge = 12;
static const int position_construction_go = 13;
static const int hover_to_stable_construction = 14;
static const int place_point_get_close = 15;
static const int place_point_adjust = 16;
static const int componnet_place = 17;
static const int place_done = 18;
static const int construction_leave = 19;
static const int place_status_judge = 20;
static const int force_return_home = 21;
static const int return_home = 22;
static const int land = 23;

//for box lost process
static const int hover_after_lost = 25;
static const int construct_point_adjust = 21;


//mission 
int loop = 0;
int current_mission_state = takeoff;
int last_mission_state = takeoff;
//time
ros::Time mission_timer_start_time;
ros::Time mission_last_time;
bool mission_timer_enable = true;

//land
state_machine::CommandTOL landing_cmd;
ros::ServiceClient land_client;
ros::Time landing_last_request;

//average of vision position
// int vision_count = 0;
// float position_x_average = 0;
// float position_y_average = 0;

bool force_home_enable = true;

/***************************callback function definition***************/
state_machine::State current_state;
state_machine::State last_state;
void state_cb(const state_machine::State::ConstPtr& msg)
{
    last_state = current_state;
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

state_machine::TASK_STATUS_CHANGE_P2M task_status_change;
void task_status_change_p2m_cb(const state_machine::TASK_STATUS_CHANGE_P2M::ConstPtr& msg)
{
    task_status_change = *msg;
    current_mission_state = task_status_change.task_status;
    loop = task_status_change.loop;
}

state_machine::FIXED_TARGET_POSITION_P2M fix_target_position;
void fixed_target_position_p2m_cb(const state_machine::FIXED_TARGET_POSITION_P2M::ConstPtr& msg)
{
    fix_target_position = *msg;

    fix_target_return.home_x = fix_target_position.home_x;
    fix_target_return.home_y = fix_target_position.home_y;
    fix_target_return.home_z = TAKEOFF_HEIGHT;

    fix_target_return.component_x = fix_target_position.component_x;
    fix_target_return.component_y = fix_target_position.component_y;
    fix_target_return.component_z = OBSERVE_HEIGET;

    fix_target_return.construction_x = fix_target_position.construction_x;
    fix_target_return.construction_y = fix_target_position.construction_y;
    fix_target_return.construction_z = OBSERVE_HEIGET;

    ROS_INFO("fix_target_return.home_x:%f",fix_target_return.home_x);
    ROS_INFO("fix_target_return.home_y:%f",fix_target_return.home_y);
    ROS_INFO("fix_target_return.home_z:%f",fix_target_return.home_z);

    ROS_INFO("fix_target_return.component_x:%f",fix_target_return.component_x);
    ROS_INFO("fix_target_return.component_y:%f",fix_target_return.component_y);
    ROS_INFO("fix_target_return.component_z:%f",fix_target_return.component_z);

    ROS_INFO("fix_target_return.construction_x:%f",fix_target_return.construction_x);
    ROS_INFO("fix_target_return.construction_y:%f",fix_target_return.construction_y);
    ROS_INFO("fix_target_return.construction_z:%f",fix_target_return.construction_z);

    fixed_target_pub.publish(fix_target_return);

    //Coordinate transformation
    //position of home
    position_home.pose.position.x = fix_target_position.home_y;
    position_home.pose.position.y = fix_target_position.home_x;
    position_home.pose.position.z = TAKEOFF_HEIGHT;
    //position of componnet
    position_componnet.pose.position.x = fix_target_position.component_y;
    position_componnet.pose.position.y = fix_target_position.component_x;
    position_componnet.pose.position.z = OBSERVE_HEIGET;
    //position of construction
    position_construction.pose.position.x = fix_target_position.construction_y;
    position_construction.pose.position.y = fix_target_position.construction_x;
    position_construction.pose.position.z = CONSTRUCT_HEIGET;
    
    //adjust angular,face north
    float yaw_sp=M_PI_2;
    //home
    position_home.pose.orientation.x = 0;
    position_home.pose.orientation.y = 0;
    position_home.pose.orientation.z = sin(yaw_sp/2);
    position_home.pose.orientation.w = cos(yaw_sp/2);
    //componnet
    position_componnet.pose.orientation.x = 0;
    position_componnet.pose.orientation.y = 0;
    position_componnet.pose.orientation.z = sin(yaw_sp/2);
    position_componnet.pose.orientation.w = cos(yaw_sp/2);
    //construction
    position_construction.pose.orientation.x = 0;
    position_construction.pose.orientation.y = 0;
    position_construction.pose.orientation.z = sin(yaw_sp/2);
    position_construction.pose.orientation.w = cos(yaw_sp/2);
}


state_machine::Vision_Position_Raw vision_position_raw;
void vision_position_cb(const state_machine::Vision_Position_Raw::ConstPtr& msg)
{
    vision_position_raw = *msg;

    vision_position_get.loop_value = loop;
    vision_position_get.component_position_x = vision_position_raw.x;
    vision_position_get.component_position_y = vision_position_raw.y;
    vision_position_get.component_position_z = vision_position_raw.z;

    vision_position_pub.publish(vision_position_get);
}

state_machine::Distance distance;
void distance_cb(const state_machine::Distance::ConstPtr& msg)
{
    distance = *msg;
    ROS_INFO("distance:%f",distance.distance);
}

/*****************************main function*****************************/
int main(int argc, char **argv)
{
    ros::init(argc, argv, "offboard_node");
    ros::NodeHandle nh;

    //takeoff velocity
    vel_ascend.twist.linear.x = 0.0f;
    vel_ascend.twist.linear.y = 0.0f;
    vel_ascend.twist.linear.z = TAKEOFF_VELOCITY;
    vel_ascend.twist.angular.x = 0.0f;
    vel_ascend.twist.angular.y = 0.0f;
    vel_ascend.twist.angular.z = 0.0f;

    //descend to grab and place velocity
    vel_descend.twist.linear.x = 0.0f;
    vel_descend.twist.linear.y = 0.0f;
    vel_descend.twist.linear.z = -DESCEND_VELOCITY;
    vel_descend.twist.angular.x = 0.0f;
    vel_descend.twist.angular.y = 0.0f;
    vel_descend.twist.angular.z = 0.0f;

    //topic  subscribe
    ros::Subscriber state_sub = nh.subscribe<state_machine::State>("mavros/state",10,state_cb);
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose",10,pose_cb);
    ros::Subscriber vel_sub = nh.subscribe<geometry_msgs::TwistStamped>("mavros/local_position/velocity",10,velo_cb);
    ros::Subscriber fixed_target_sub = nh.subscribe<state_machine::FIXED_TARGET_POSITION_P2M>("mavros/fixed_target_position_p2m",10,fixed_target_position_p2m_cb);
    ros::Subscriber task_status_sub = nh.subscribe<state_machine::TASK_STATUS_CHANGE_P2M>("mavros/task_status_change_p2m",10,task_status_change_p2m_cb);
    ros::Subscriber distance_sub = nh.subscribe<state_machine::Distance>("distance",10,distance_cb);
    ros::Subscriber vision_position_sub = nh.subscribe<state_machine::Vision_Position_Raw>("vision_position",10,vision_position_cb);

    //topic publish
    local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local",10);
    local_vel_pub = nh.advertise<geometry_msgs::TwistStamped>("mavros/setpoint_velocity/cmd_vel",10);
    fixed_target_pub = nh.advertise<state_machine::FIXED_TARGET_RETURN_M2P>("mavros/fixed_target_return_m2p",10);
    ros::Publisher grab_status_pub = nh.advertise<state_machine::GRAB_STATUS_M2P>("mavros/grab_status_m2p",10);
    ros::Publisher task_status_pub = nh.advertise<state_machine::TASK_STATUS_MONITOR_M2P>("mavros/task_status_monitor_m2p",10);
    vision_position_pub = nh.advertise<state_machine::VISION_POSITION_GET_M2P>("mavros/vision_position_get_m2p",10);
    ros::Publisher yaw_sp_pub = nh.advertise<state_machine::YAW_SP_CALCULATED_M2P>("mavros/yaw_sp_calculated_m2p",10);

    //land service
    land_client = nh.serviceClient<state_machine::CommandTOL>("mavros/cmd/land");
    landing_cmd.request.min_pitch = 1.0;
    landing_last_request = ros::Time::now();

    ros::Rate rate(10.0);

    while(ros::ok() && !current_state.connected)
    {
    	ros::spinOnce();
    	rate.sleep();
    }

    ROS_INFO("Connect successfully!!");

    ROS_INFO("send setpoint before takeoff,please wait");

    for(int i =100; ros::ok() && i > 0; i--)
	{
		local_vel_pub.publish(vel_ascend);
		ros::spinOnce();
		rate.sleep();
	}

    ROS_INFO("Initialization finished");

	while(ros::ok())
	{
        state_machine_fun();
        ROS_INFO("current_mission_state:%d",current_mission_state);
        ROS_INFO("loop:%d",loop);

        task_status_monitor.task_status = current_mission_state;
        task_status_monitor.loop_value = loop;
        task_status_monitor.target_x = pose_target.pose.position.y;
        task_status_monitor.target_y = pose_target.pose.position.x;
        task_status_monitor.target_z = -pose_target.pose.position.z;
        task_status_pub.publish(task_status_monitor);

        if(ros::Time::now() - mission_timer_start_time > ros::Duration(MAX_MISSION_TIME) && force_home_enable == true)
        {
            force_home_enable = false;
            current_mission_state = force_return_home;
            mission_last_time = ros::Time::now();
            ROS_INFO("Mission time run out,will return home and landing!!");
        }
        
        if(distance.distance > 1.0)
        {
            if(current_mission_state == position_construction_go)
            {
                position_after_lose.pose.position.x = current_position.pose.position.x;
                position_after_lose.pose.position.y = current_position.pose.position.y;
                position_after_lose.pose.position.z = current_position.pose.position.z;
                last_mission_state = position_construction_go;
                current_mission_state = hover_after_lost;
                mission_last_time = ros::Time::now();
            }
            if(current_mission_state == hover_to_stable_construction)
            {
                position_after_lose.pose.position.x = current_position.pose.position.x;
                position_after_lose.pose.position.y = current_position.pose.position.y;
                position_after_lose.pose.position.z = current_position.pose.position.z;
                last_mission_state = hover_to_stable_construction;
                current_mission_state = hover_after_lost;
                mission_last_time = ros::Time::now();
            }
            if(current_mission_state == place_point_get_close)
            {
                position_after_lose.pose.position.x = current_position.pose.position.x;
                position_after_lose.pose.position.y = current_position.pose.position.y;
                position_after_lose.pose.position.z = current_position.pose.position.z;
                last_mission_state = place_point_get_close;
                current_mission_state = hover_after_lost;
                mission_last_time = ros::Time::now();
            }

            grab_status.grab_status = 0;
            grab_status_pub.publish(grab_status);
        }
        else if(current_mission_state == componnet_get_close || current_mission_state == componnet_grab)
        {
            grab_status.grab_status = 0;
            grab_status_pub.publish(grab_status);
        }
        else
        {
            grab_status.grab_status = 1;
            grab_status_pub.publish(grab_status);
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
            pose_target = position_home;
            local_vel_pub.publish(vel_ascend);
            if(mission_timer_enable)
            {
                mission_timer_start_time = ros::Time::now();
                mission_timer_enable = false;
            }
            if(current_velocity.twist.linear.z > 0.5 && current_position.pose.position.z > 2)
            {
                current_mission_state = position_home_go;
                mission_last_time = ros::Time::now();
            }
        }
        break;
        case position_home_go:
        {
            pose_target = position_home;
            local_pos_pub.publish(position_home);
            if (Distance_of_Two(current_position.pose.position.x,position_home.pose.position.x,
                                current_position.pose.position.y,position_home.pose.position.y,
                                current_position.pose.position.z,position_home.pose.position.z) < 0.2)
            {
                current_mission_state = position_home_hover;
                mission_last_time = ros::Time::now();
            }
        }
        break;
        case position_home_hover:
        {
            pose_target = position_home;
            local_pos_pub.publish(position_home);
            if(ros::Time::now() - mission_last_time > ros::Duration(2.0))
            {
                current_mission_state = position_component_go;
                mission_last_time = ros::Time::now();
            }
        }
        break;
        case position_component_go:
        {
            pose_target = position_componnet;
            local_pos_pub.publish(position_componnet);
            if (Distance_of_Two(current_position.pose.position.x,position_componnet.pose.position.x,
                                current_position.pose.position.y,position_componnet.pose.position.y,
                                current_position.pose.position.z,position_componnet.pose.position.z) < 0.2)
            {
                current_mission_state = hover_to_recognize;
                mission_last_time = ros::Time::now();
            }
        }
        break;
        case hover_to_recognize:
        {
            static int vision_count = 11;
            static int row_count_x_max = 0;
            static int row_count_x_min = 0;
            static int row_count_y_max = 0;
            static int row_count_y_min = 0;
            static float position_x_max = 0;
            static float position_y_max = 0;
            static float position_x_min = 0;
            static float position_y_min = 0;
            static float position_x_total = 0;
            static float position_y_total = 0;
            static float position_x_average = 0;
            static float position_y_average = 0;
            static float position[10][2] = {0};
            
            pose_target = position_componnet;
            local_pos_pub.publish(position_componnet);
            if (ros::Time::now() - mission_last_time > ros::Duration(1.0) && vision_count > 10)
            {
                vision_count = 0;
            }
            if(ros::Time::now() - mission_last_time > ros::Duration(0.5) && vision_count < 10)
            {
                // position_x_average = (position_x_average * vision_count + vision_position.component_position_x)/(vision_count);
                // position_y_average = (position_x_average * vision_count + vision_position.component_position_y)/(vision_count);
                if(vision_position_get.component_position_x > position_x_max)
                {
                    position_x_max = vision_position_get.component_position_x;
                    row_count_x_max = vision_count;
                }
                if(vision_position_get.component_position_y > position_y_max)
                {
                    position_y_max = vision_position_get.component_position_y;
                    row_count_y_max = vision_count;
                }
                if(vision_position_get.component_position_x < position_x_min)
                {
                    position_x_min = vision_position_get.component_position_x;
                    row_count_x_min = vision_count;
                }
                if(vision_position_get.component_position_y < position_y_min)
                {
                    position_y_min = vision_position_get.component_position_y;
                    row_count_y_min = vision_count;
                }
                position[vision_count][0] = vision_position_get.component_position_x;
                position[vision_count][1] = vision_position_get.component_position_y;
                position_x_max = position[0][0];
                position_y_max = position[0][1];
                position_x_min = position[0][0];
                position_y_min = position[0][1];
                vision_count++;
            }
            else
            {
                for(int i = 0;i < 10;i++)
                    if(i != row_count_x_max && i != row_count_x_min)
                        position_x_total += position[i][0];
                for(int i = 0;i < 10;i++)
                    if(i != row_count_x_max && i != row_count_x_min)
                        position_y_total += position[i][1];
                position_x_average = position_x_total/8;
                position_y_average = position_y_total/8;

                position_by_calculated.pose.position.x = position_x_average;
                position_by_calculated.pose.position.y = position_y_average;
                position_by_calculated.pose.position.z = 2.5;

                vision_count = 11;
                row_count_x_max = 0;
                row_count_x_min = 0;
                row_count_y_max = 0;
                row_count_y_min = 0;
                position_x_average = 0;
                position_y_average = 0;
                position_x_max = 0;
                position_y_max = 0;
                position_x_min = 0;
                position_y_min = 0;
                position_x_total = 0;
                position_y_total = 0;
                static float position[10][2] = {0};

                current_mission_state = component_locate;
                mission_last_time = ros::Time::now();
            }
        }
        break;
        case component_locate:
        {
            pose_target = position_by_calculated;
            local_pos_pub.publish(position_by_calculated);
            if (Distance_of_Two(current_position.pose.position.x,position_componnet.pose.position.x,
                                current_position.pose.position.y,position_componnet.pose.position.y,
                                current_position.pose.position.z,position_componnet.pose.position.z) < 0.2)
            {
                current_mission_state = hover_to_stable_component;
                mission_last_time = ros::Time::now();
            }
        }
        break;
        case hover_to_stable_component:
        {
            static int accuracy_count = 0;  //for improve accuracy
            static int hover_count = 0;
            pose_target = position_by_calculated;
            local_pos_pub.publish(position_by_calculated);
            if(ros::Time::now() - mission_last_time > ros::Duration(2.0) && hover_count == 0)
            {
                hover_count++;
            }
            if (ros::Time::now() - mission_last_time > ros::Duration(0.5) && hover_count > 0)
            {
                if (Distance_of_Two(current_position.pose.position.x,position_componnet.pose.position.x,
                                    current_position.pose.position.y,position_componnet.pose.position.y,
                                    current_position.pose.position.z,position_componnet.pose.position.z) < 0.1)
                {
                    accuracy_count++;
                    if(accuracy_count > 3)
                    {
                        current_mission_state = component_get_close;
                        accuracy_count = 0;
                        hover_count = 0;
                        mission_last_time = ros::Time::now();
                        break;
                    }
                }
                else
                {
                    accuracy_count = 0;
                }
            }           
            hover_count++;
            if(hover_count > 10)
            {
                current_mission_state = component_get_close;
                accuracy_count = 0;
                hover_count = 0;
                mission_last_time = ros::Time::now();
                break;
            }
        }
        break;
        case component_get_close:
        {
            pose_target = position_by_calculated;
            local_vel_pub.publish(vel_descend);
            if (abs(current_position.pose.position.x - position_componnet.pose.position.x) < 0.1 &&
                abs(current_position.pose.position.y - position_componnet.pose.position.y) < 0.1 &&
                distance.distance < 0.5 )
            {
                current_mission_state = component_grab;

                position_to_grab.pose.position.x = position_by_calculated.pose.position.x;
                position_to_grab.pose.position.y = position_by_calculated.pose.position.y;
                position_to_grab.pose.position.z = current_position.pose.position.z;
        
                mission_last_time = ros::Time::now();
            }
        }
        break;
        case component_grab:
        {
            static int grab_count = 0;
            pose_target = position_to_grab;
            local_pos_pub.publish(position_to_grab);
            if(ros::Time::now() - mission_last_time > ros::Duration(1.0) && grab_count == 0)
            {
                grab_count++;
            }
            if(ros::Time::now() - mission_last_time > ros::Duration(0.5) && grab_count > 0)
            {
                grab_count++;
                if(grab_count > 40)
                {
                    grab_count = 0;
                    current_mission_state = component_leave;
                    mission_last_time = ros::Time::now();
                    break;
                }
            }
                
        }
        break;
        case component_leave:
        {
            pose_target = position_to_grab;
            local_vel_pub.publish(vel_ascend);
            if (current_position.pose.position.z > 3)
            {
                position_to_judge.pose.position.x = current_position.pose.position.x;
                position_to_judge.pose.position.y = current_position.pose.position.y;
                position_to_judge.pose.position.z = current_position.pose.position.z;
                current_mission_state = grab_status_judge;
                mission_last_time = ros::Time::now();
            }
        }
        break;
        case grab_status_judge:
        {
            static int grab_judge_count = 0;
            pose_target = position_to_judge;
            local_pos_pub.publish(position_to_judge);
            if(ros::Time::now() - mission_last_time > ros::Duration(1.0) && grab_judge_count == 0)
            {
                grab_judge_count++;
            }
            if (ros::Time::now() - mission_last_time > ros::Duration(0.5) && grab_judge_count >0)
            {
                if (distance.distance < 0.5)
                    {
                        grab_judge_count++;
                        if(grab_judge_count > 10)
                        {
                            grab_judge_count = 0;
                            current_mission_state = position_construction_go;
                            mission_last_time = ros::Time::now();
                            break;
                        }
                    }
                else
                {
                    grab_judge_count = 0;
                    current_mission_state = position_component_go;
                    mission_last_time = ros::Time::now();
                    break;
                }               
            }
        }
        break;
        case position_construction_go:
        {
            pose_target = position_construction;
            local_pos_pub.publish(position_construction);
            if (Distance_of_Two(current_position.pose.position.x,position_construction.pose.position.x,
                                current_position.pose.position.y,position_construction.pose.position.y,
                                current_position.pose.position.z,position_construction.pose.position.z) < 0.2)
            {
                current_mission_state = hover_to_stable_construction;
                mission_last_time = ros::Time::now();
            }
        }
        break;
        case hover_to_stable_construction:
        {
            pose_target = position_construction;
            local_pos_pub.publish(position_construction);
            if(ros::Time::now() - mission_last_time > ros::Duration(2.0) && hover_count == 0)
            {
                hover_count++;
            }
            if (ros::Time::now() - mission_last_time > ros::Duration(0.5) && hover_count > 0)
            {
                if (Distance_of_Two(current_position.pose.position.x,position_construction.pose.position.x,
                                    current_position.pose.position.y,position_construction.pose.position.y,
                                    current_position.pose.position.z,position_construction.pose.position.z) < 0.1)
                {
                    accuracy_count++;
                    if(accuracy_count > 3)
                    {
                        current_mission_state = place_point_get_close;
                        accuracy_count = 0;
                        hover_count = 0;

                        position_to_place.pose.position.x = position_construction.pose.position.x;
                        position_to_place.pose.position.y = position_construction.pose.position.y;
                        position_to_place.pose.position.z = PLACE_HEIGET + BOX_HEIGET*loop;

                        mission_last_time = ros::Time::now();
                        break;
                    }
                }
                else
                {
                    accuracy_count = 0;
                }
            }           
            hover_count++;
            if(hover_count > 10)
            {
                current_mission_state = place_point_get_close;
                accuracy_count = 0;
                hover_count = 0;
                
                position_to_place.pose.position.x = position_construction.pose.position.x;
                position_to_place.pose.position.y = position_construction.pose.position.y;
                position_to_place.pose.position.z = PLACE_HEIGET + BOX_HEIGET*loop;

                mission_last_time = ros::Time::now();
                break;
            }
        }
        break;
        case place_point_get_close:
        {
            pose_pub = position_to_place;
            local_pos_pub.publish(position_to_place);
            if (Distance_of_Two(current_position.pose.position.x,position_to_place.pose.position.x,
                                current_position.pose.position.y,position_to_place.pose.position.y,
                                current_position.pose.position.z,position_to_place.pose.position.z) < 0.2 )
            {
                current_mission_state = place_point_adjust;
                mission_last_time = ros::Time::now();
            }
        }
        break;
        case place_point_adjust:
        {
            pose_target = position_to_place;
            local_pos_pub.publish(position_to_place);
            if(ros::Time::now() - mission_last_time > ros::Duration(2.0) && hover_count == 0)
            {
                hover_count++;
            }
            if (ros::Time::now() - mission_last_time > ros::Duration(0.5) && hover_count > 0)
            {
                if (Distance_of_Two(current_position.pose.position.x,position_to_place.pose.position.x,
                                    current_position.pose.position.y,position_to_place.pose.position.y,
                                    current_position.pose.position.z,position_to_place.pose.position.z) < 0.1)
                {
                    accuracy_count++;
                    if(accuracy_count > 3)
                    {
                        current_mission_state = componnet_place;
                        accuracy_count = 0;
                        hover_count = 0;
                        mission_last_time = ros::Time::now();
                        break;
                    }
                }
                else
                {
                    accuracy_count = 0;
                }
            }           
            hover_count++;
            if(hover_count > 10)
            {
                current_mission_state = componnet_place;
                accuracy_count = 0;
                hover_count = 0;
                mission_last_time = ros::Time::now();
                break;
            }
        }
        break;
        case componnet_place:
        {
            static int place_count = 0;
            pose_target = position_to_place;
            local_pos_pub.publish(position_to_place);
            if(ros::Time::now() - mission_last_time > ros::Duration(1.0) && place_count == 0)
            {
                place_count++;
            }
            if(ros::Time::now() - mission_last_time > ros::Duration(0.5) && place_count > 0)
            {
                place_count++;
                if(place_count > 40)
                {
                    place_count = 0;
                    current_mission_state = component_leave;
                    mission_last_time = ros::Time::now();
                    break;
                }
            }
        }
        break;
        case component_leave:
        {
            pose_target = position_place;
            local_vel_pub.publish(vel_ascend);
            if (current_position.pose.position.z > (3 + BOX_HEIGET*loop))
            {
                position_to_component.pose.position.x = current_position.pose.position.x;
                position_to_component.pose.position.y = current_position.pose.position.y;
                position_to_component.pose.position.z = current_position.pose.position.z;
                current_mission_state = place_status_judge;
                mission_last_time = ros::Time::now();
            }
        }
        break;
        case place_status_judge:
        {
            static int place_judge_count = 0;
            pose_target = position_to_component;
            local_pos_pub.publish(position_to_component);
            if(ros::Time::now() - mission_last_time > ros::Duration(1.0) && place_judge_count == 0)
            {
                place_judge_count++;
            }
            if (ros::Time::now() - mission_last_time > ros::Duration(0.5) && place_judge_count >0)
            {
                if (distance.distance > 1.0)
                    {
                        place_judge_count++;
                        if(place_judge_count > 3)
                        {
                            place_judge_count = 0;
                            current_mission_state = position_component_go;
                            loop++;
                            mission_last_time = ros::Time::now();
                            break;
                        }
                    }
                else
                {
                    place_judge_count = 0;
                    current_mission_state = place_point_get_close;
                    mission_last_time = ros::Time::now();
                    break;
                }               
            }
        }
        break;
        case hover_after_lost:
        {
            pose_target = position_after_lose;
            local_pos_pub.publish(position_after_lose);
            if (ros::Time::now() - mission_last_time > ros::Duration(2.0))
            {
                if(last_mission_state == position_construction_go)
                {
                    current_mission_state = position_component_go;
                    mission_last_time = ros::Time::now();
                    break;
                }
                if(last_mission_state == hover_to_stable_construction || place_point_get_close)
                {
                    current_mission_state = construct_point_adjust;
                    mission_last_time = ros::Time::now();
                    break;
                }
            }
        }
        break;
        case construct_point_adjust:
        {
            pose_target = position_after_lose;
            local_pos_pub.publish(position_after_lose);
            position_construction.pose.position.x = position_construction.pose.position.x + 2;
            position_construction.pose.position.y = position_construction.pose.position.y + 2;
            position_construction.pose.position.z = position_construction.pose.position.z;
            current_mission_state = position_component_go;
            last_time = ros::Time::now();
        }
        break;
        case force_return_home:
        {
            position_return_home.pose.position.x = current_position.pose.position.x;
            position_return_home.pose.position.y = current_position.pose.position.y;
            position_return_home.pose.position.z = current_position.pose.position.z;
            current_mission_state = return_home;
            mission_last_time = ros::Time::now();
        }
        break;
        case  return_home:
        {
            pose_target = position_home;
            local_pos_pub.publish(position_home);
            if (Distance_of_Two(current_position.pose.position.x,position_home.pose.position.x,
                                current_position.pose.position.y,position_home.pose.position.y,
                                current_position.pose.position.z,position_home.pose.position.z) < 0.2 )
            {
                current_mission_state = hover_only;
                mission_last_time = ros::Time::now();
            }
        }
        break;
        case hover_only:
        {
            pose_target = position_home;
            local_pos_pub.publish(position_home);
            if(ros::Time::now() - mission_last_time >ros::Duration(3.0))
            {
                current_mission_state = land;
                mission_last_time = ros::Time::now();
            }
        }
        break;
        case land:
        {
            if(current_state.mode != "AUTO.LAND" && 
                   (ros::Time::now() - landing_last_request > ros::Duration(10.0)))
                {
                    if(land_client.call(landing_cmd) && landing_cmd.response.success)
                    {
                        ROS_INFO("AUTO LANDING");
                    }
                    landing_last_request = ros::Time::now();
                }
        }
        break;
    }
}


/**************************************function definition**************************************/
//Euclidean distance between two point
double Distance_of_Two(double x1, double x2, double y1, double y2, double z1, double z2)
{
    return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1)+(z2-z1)*(z2-z1));
}

// limit angle_rad to [-pi,pi]
float wrap_pi(float angle_rad)
{
    //value is inf or NaN
    //the macro definition of M_PI and NAN are both in <math.h>
    //M_PI means 3.14159265257...,NAN means not define

    int c = 0;

    if (angle_rad > 10 || angle_rad < -10) 
    {
        return angle_rad;
    }
    
    while (angle_rad >= M_PI) 
    {
        angle_rad -= M_PI*2;

        if (c++ > 3) 
        {
            return NAN;
        }
    }
    c = 0;
    while (angle_rad < -M_PI) 
    {
        angle_rad += M_PI*2;

        if (c++ > 3) 
        {
            return NAN;
        }
    }
    return angle_rad;
}