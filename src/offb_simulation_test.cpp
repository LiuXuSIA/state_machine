 /**
* @file     : offb_simulation_test.cpp
* @brief    : offboard simulation test: demo rewritten -> 4 setpoints flight -> complete state machine.
* @author   : libn
* @Time     : Aug 25, 201610:06:42 PM
*/

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>			/* local velocity setpoint. -libn */
#include <state_machine/CommandBool.h>
#include <state_machine/SetMode.h>
#include <state_machine/State.h>
#include <state_machine/CommandTOL.h>
#include <state_machine/Setpoint.h>
#include <state_machine/DrawingBoard10.h>

/* subscribe messages from pixhawk. -libn */
#include <state_machine/FIXED_TARGET_POSITION_P2M.h>
#include <state_machine/TASK_STATUS_CHANGE_P2M.h>

/* publish messages to pixhawk. -libn */
#include <state_machine/FIXED_TARGET_RETURN_M2P.h>
#include <state_machine/OBSTACLE_POSITION_M2P.h>
#include <state_machine/TASK_STATUS_MONITOR_M2P.h>
#include <state_machine/VISION_NUM_SCAN_M2P.h>
#include <state_machine/VISION_ONE_NUM_GET_M2P.h>
#include <state_machine/YAW_SP_CALCULATED_M2P.h>
#define SPRAY_DISTANCE 2  /* distance from UAV to drawing board while sparying. */
#define VISION_SCAN_DISTANCE 2.5  /* distance from UAV to drawing board while hoveing and scanning. */

#define SAFE_HEIGHT_DISTANCE 0.4  /* distanche from drawing board's height to expected height: 0: real mission; >0: for safe. */
#define FIXED_POS_HEIGHT 1.5    /* height of point: H,O,L,R. */

#include <math.h>

#include <std_msgs/Int32.h>

#define MAX_FLIGHT_TIME 220 /* max flight time of whole mission. */

#include <state_machine/FailureRecord.h>
#define FAILURE_REPAIR 1    /* FAILURE_REPAIR: 0: never repair errores; 1: repair errors. */

void state_machine_func(void);
/* mission state. -libn */
static const int takeoff = 1;
static const int mission_hover_after_takeoff = 2;
static const int mission_hover_only = 3;
static const int mission_observe_point_go = 5;
static const int mission_observe_num_wait = 6;
static const int mission_num_search = 8;
static const int mission_num_scan_again = 9;
static const int mission_num_locate = 10;
static const int mission_num_get_close = 11;
static const int mission_arm_spread = 12;
static const int mission_num_hover_spray = 13;
static const int mission_num_done = 14;
static const int mission_return_home = 15;
static const int land = 16;
static const int mission_end = 17;
static const int mission_hover_before_spary = 18;
static const int mission_fix_failure = 19;
static const int mission_hover_after_stretch_back = 20;
static const int mission_force_return_home = 21;

int loop = 1;	/* loop calculator: loop = 1/2/3/4/5. -libn */
// current mission state, initial state is to takeoff
int current_mission_state = takeoff;
ros::Time mission_last_time;	/* timer used in mission. -libn */
bool display_screen_num_recognized = false;	/* to check if the num on display screen is recognized. -libn */
bool relocate_valid = false;	/* to complete relocate mission. -libn */

int current_mission_num;	/* mission num: 5 subtask -> 5 current nums.	TODO:change mission num. -libn */
int last_mission_num;

bool velocity_control_enable = true;

ros::Time mission_timer_start_time;	/* timer to control the whole mission and 5 subtasks. -libn */
bool mission_timer_enable = true;   /* start mission_timer. */
bool force_home_enable = true;

/* 4 setpoints. -libn */
geometry_msgs::PoseStamped setpoint_A;
geometry_msgs::PoseStamped setpoint_L;
geometry_msgs::PoseStamped setpoint_R;
geometry_msgs::PoseStamped setpoint_D;
geometry_msgs::PoseStamped setpoint_H;	/* home position. -libn */

state_machine::FailureRecord failure[5];
int mission_failure_acount = 0;

state_machine::State current_state;
state_machine::State last_state;
state_machine::State last_state_display;
void state_cb(const state_machine::State::ConstPtr& msg){
	last_state_display.mode = current_state.mode;
	last_state_display.armed = current_state.armed;
	current_state = *msg;
}

state_machine::YAW_SP_CALCULATED_M2P yaw_sp_calculated_m2p_data,yaw_sp_pub2GCS;
geometry_msgs::PoseStamped pose_pub;
geometry_msgs::TwistStamped vel_pub;	/* velocity setpoint to be published. -libn */

/* limit angle_rad to [-pi,pi]. */
float wrap_pi(float angle_rad)
{
    /* value is inf or NaN */
    if (angle_rad > 10 || angle_rad < -10) {
        return angle_rad;
    }
    int c = 0;
    while (angle_rad >= M_PI) {
        angle_rad -= M_PI*2;

        if (c++ > 3) {
            return NAN;
        }
    }
    c = 0;
    while (angle_rad < -M_PI) {
        angle_rad += M_PI*2;

        if (c++ > 3) {
            return NAN;
        }
    }
    return angle_rad;
}

ros::Publisher  yaw_sp_calculated_m2p_pub;

float deta_x,deta_y;
state_machine::Setpoint setpoint_indexed;
/* get 4 setpoints and calculate yaw*. */
void SetpointIndexedCallback(const state_machine::Setpoint::ConstPtr& msg)
{
	setpoint_indexed = *msg;
    /* get 4 setpoints:A,B,C,D(ENU). -libn */
    switch(setpoint_indexed.index)
    {
        case 1:
            setpoint_A.pose.position.x = setpoint_indexed.x;
            setpoint_A.pose.position.y = setpoint_indexed.y;
//            setpoint_A.pose.position.z = setpoint_indexed.z;
            break;
        case 2:
            setpoint_L.pose.position.x = setpoint_indexed.x;
            setpoint_L.pose.position.y = setpoint_indexed.y;
//            setpoint_L.pose.position.z = setpoint_indexed.z;
            break;
        case 3:
            setpoint_R.pose.position.x = setpoint_indexed.x;
            setpoint_R.pose.position.y = setpoint_indexed.y;
//            setpoint_R.pose.position.z = setpoint_indexed.z;
            break;
        case 4:
            setpoint_D.pose.position.x = setpoint_indexed.x;    /* not used! */
            setpoint_D.pose.position.y = setpoint_indexed.y;
//            setpoint_D.pose.position.z = setpoint_indexed.z;
            break;
        default:
            ROS_INFO("setpoint index error!");
            break;
    }

    /* calculate yaw*. -libn */
    deta_x = setpoint_R.pose.position.x - setpoint_L.pose.position.x;
    deta_y = setpoint_R.pose.position.y - setpoint_L.pose.position.y;
    yaw_sp_calculated_m2p_data.yaw_sp = atan2(deta_y,deta_x);
    yaw_sp_calculated_m2p_data.yaw_sp = wrap_pi(yaw_sp_calculated_m2p_data.yaw_sp + M_PI/2);    /* yaw* in ENU in rad within [-pi,pi]. */
    ROS_INFO("yaw*(ENU) calculated for test with send4setpoint running.");
    /* yaw* for controller. */
    pose_pub.pose.orientation.x = 0;			/* orientation expressed using quaternion. -libn */
    pose_pub.pose.orientation.y = 0;			/* w = cos(theta/2), x = nx * sin(theta/2),  y = ny * sin(theta/2), z = nz * sin(theta/2) -libn */
    pose_pub.pose.orientation.z = sin(yaw_sp_calculated_m2p_data.yaw_sp/2);
    pose_pub.pose.orientation.w = cos(yaw_sp_calculated_m2p_data.yaw_sp/2);		/* set yaw* = 90 degree(default in simulation). -libn */

    /* publish yaw_sp to pixhawk. */
    yaw_sp_calculated_m2p_pub.publish(yaw_sp_calculated_m2p_data);
    ROS_INFO("publishing yaw_sp_calculated_m2p(ENU): %f",
            yaw_sp_calculated_m2p_data.yaw_sp);

}

// local position msg callback function
geometry_msgs::PoseStamped current_pos;
void pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    current_pos = *msg;
}

// local velocity msg callback function
geometry_msgs::TwistStamped current_vel;
void vel_cb(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    current_vel = *msg;
//    ROS_INFO("Vx = %f Vy = %f Vz = %f",current_vel.twist.linear.x,current_vel.twist.linear.y,current_vel.twist.linear.z);
}

/* 10 drawing board positions. -libn */
state_machine::DrawingBoard10 board10;
void board_pos_cb(const state_machine::DrawingBoard10::ConstPtr& msg)
{
	board10 = *msg;

//	ROS_INFO("\nboard_0 position: %d x = %f y = %f z = %f\n"
//				"board_1 position: %d x = %f y = %f z = %f\n"
//				"board_2 position: %d x = %f y = %f z = %f\n"
//				"board_3 position: %d x = %f y = %f z = %f\n"
//				"board_4 position: %d x = %f y = %f z = %f\n"
//				"board_5 position: %d x = %f y = %f z = %f\n"
//				"board_6 position: %d x = %f y = %f z = %f\n"
//				"board_7 position: %d x = %f y = %f z = %f\n"
//				"board_8 position: %d x = %f y = %f z = %f\n"
//				"board_9 position: %d x = %f y = %f z = %f\n",
//				board10.drawingboard[0].valid,board10.drawingboard[0].x,board10.drawingboard[0].y,board10.drawingboard[0].z,
//				board10.drawingboard[1].valid,board10.drawingboard[1].x,board10.drawingboard[1].y,board10.drawingboard[1].z,
//				board10.drawingboard[2].valid,board10.drawingboard[2].x,board10.drawingboard[2].y,board10.drawingboard[2].z,
//				board10.drawingboard[3].valid,board10.drawingboard[3].x,board10.drawingboard[3].y,board10.drawingboard[3].z,
//				board10.drawingboard[4].valid,board10.drawingboard[4].x,board10.drawingboard[4].y,board10.drawingboard[4].z,
//				board10.drawingboard[5].valid,board10.drawingboard[5].x,board10.drawingboard[5].y,board10.drawingboard[5].z,
//				board10.drawingboard[6].valid,board10.drawingboard[6].x,board10.drawingboard[6].y,board10.drawingboard[6].z,
//				board10.drawingboard[7].valid,board10.drawingboard[7].x,board10.drawingboard[7].y,board10.drawingboard[7].z,
//				board10.drawingboard[8].valid,board10.drawingboard[8].x,board10.drawingboard[8].y,board10.drawingboard[8].z,
//				board10.drawingboard[9].valid,board10.drawingboard[9].x,board10.drawingboard[9].y,board10.drawingboard[9].z);
}


ros::Publisher  fixed_target_return_m2p_pub;

/* transform to ENU from NED. */
void position_x_ENU_from_NED(float x_NED, float y_NED, float z_NED, float* pos_ENU_f)
{
    *pos_ENU_f = y_NED;
    *(pos_ENU_f+1) = x_NED;
    *(pos_ENU_f+2) = -z_NED;
}

/* limit the error between (x2,y2) and (x1,y1). */
void error_limit(double x1, double y1, double x2, double y2, double* result)
{
    *result = (x2-x1)/sqrt(fabs((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1)));
    *(result+1) = (y2-y1)/sqrt(fabs((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1)));
}


/* subscribe messages from pixhawk. -libn */
state_machine::FIXED_TARGET_POSITION_P2M fixed_target_position_p2m_data;
state_machine::FIXED_TARGET_RETURN_M2P fixed_target_return_m2p_data;
/* get 4 setpoints and calculate yaw*. */
void fixed_target_position_p2m_cb(const state_machine::FIXED_TARGET_POSITION_P2M::ConstPtr& msg){
	fixed_target_position_p2m_data = *msg;
	ROS_INFO("subscribing fixed_target_position_p2m: %5.3f %5.3f %5.3f",
            fixed_target_position_p2m_data.home_x,
            fixed_target_position_p2m_data.home_y,
            fixed_target_position_p2m_data.home_z);

	/* publish messages to pixhawk. -libn */
    fixed_target_return_m2p_data.home_x = fixed_target_position_p2m_data.home_x;
    fixed_target_return_m2p_data.home_y = fixed_target_position_p2m_data.home_y;
    fixed_target_return_m2p_data.home_z = -FIXED_POS_HEIGHT;

    fixed_target_return_m2p_data.observe_x = fixed_target_position_p2m_data.observe_x;
    fixed_target_return_m2p_data.observe_y = fixed_target_position_p2m_data.observe_y;
    fixed_target_return_m2p_data.observe_z = -FIXED_POS_HEIGHT;

    fixed_target_return_m2p_data.spray_left_x = fixed_target_position_p2m_data.spray_left_x;
    fixed_target_return_m2p_data.spray_left_y = fixed_target_position_p2m_data.spray_left_y;
    fixed_target_return_m2p_data.spray_left_z = -FIXED_POS_HEIGHT;

    fixed_target_return_m2p_data.spray_right_x = fixed_target_position_p2m_data.spray_right_x;
    fixed_target_return_m2p_data.spray_right_y = fixed_target_position_p2m_data.spray_right_y;
    fixed_target_return_m2p_data.spray_right_z = -FIXED_POS_HEIGHT;
	fixed_target_return_m2p_pub.publish(fixed_target_return_m2p_data);
    ROS_INFO("publishing fixed_target_return_m2p(NED): %f\t%f\t%f\t",
            fixed_target_return_m2p_data.home_x,
            fixed_target_return_m2p_data.home_y,
            fixed_target_return_m2p_data.home_z);

    /* get 4 fixed_setpoint. */
    /* transform position from NED to ENU. */
    float pos_ENU[3] = {0,0,0};
    position_x_ENU_from_NED(fixed_target_position_p2m_data.home_x,
                            fixed_target_position_p2m_data.home_y,
                            fixed_target_position_p2m_data.home_z,
                            pos_ENU);
    setpoint_H.pose.position.x = pos_ENU[0];
    setpoint_H.pose.position.y = pos_ENU[1];
//    setpoint_H.pose.position.z = pos_ENU[2];

    position_x_ENU_from_NED(fixed_target_position_p2m_data.observe_x,
                            fixed_target_position_p2m_data.observe_y,
                            fixed_target_position_p2m_data.observe_z,
                            pos_ENU);
    setpoint_A.pose.position.x = pos_ENU[0];
    setpoint_A.pose.position.y = pos_ENU[1];
//    setpoint_A.pose.position.z = pos_ENU[2];

    position_x_ENU_from_NED(fixed_target_position_p2m_data.spray_left_x,
                            fixed_target_position_p2m_data.spray_left_y,
                            fixed_target_position_p2m_data.spray_left_z,
                            pos_ENU);
    setpoint_L.pose.position.x = pos_ENU[0];
    setpoint_L.pose.position.y = pos_ENU[1];
//    setpoint_L.pose.position.z = pos_ENU[2];

    position_x_ENU_from_NED(fixed_target_position_p2m_data.spray_right_x,
                            fixed_target_position_p2m_data.spray_right_y,
                            fixed_target_position_p2m_data.spray_right_z,
                            pos_ENU);
    setpoint_R.pose.position.x = pos_ENU[0];
    setpoint_R.pose.position.y = pos_ENU[1];
//    setpoint_R.pose.position.z = pos_ENU[2];

    /* calculate yaw*. -libn */
    deta_x = setpoint_R.pose.position.x - setpoint_L.pose.position.x;
    deta_y = setpoint_R.pose.position.y - setpoint_L.pose.position.y;
    yaw_sp_calculated_m2p_data.yaw_sp = atan2(deta_y,deta_x);
    yaw_sp_calculated_m2p_data.yaw_sp = wrap_pi(yaw_sp_calculated_m2p_data.yaw_sp + M_PI/2);    /* yaw* in NED in rad within [-pi,pi]. */
    ROS_INFO("yaw*(ENU) calculated using fixed_position from GCS.");
    /* yaw* for controller. */
    pose_pub.pose.orientation.x = 0;			/* orientation expressed using quaternion. -libn */
    pose_pub.pose.orientation.y = 0;			/* w = cos(theta/2), x = nx * sin(theta/2),  y = ny * sin(theta/2), z = nz * sin(theta/2) -libn */
    pose_pub.pose.orientation.z = sin(yaw_sp_calculated_m2p_data.yaw_sp/2);
    pose_pub.pose.orientation.w = cos(yaw_sp_calculated_m2p_data.yaw_sp/2);		/* set yaw* = 90 degree(default in simulation). -libn */

    /* publish yaw_sp to pixhawk. */
    yaw_sp_pub2GCS.yaw_sp = wrap_pi(-(yaw_sp_calculated_m2p_data.yaw_sp - M_PI/2));
    yaw_sp_calculated_m2p_pub.publish(yaw_sp_pub2GCS);
    ROS_INFO("publishing yaw_sp_calculated_m2p(ENU): %f",
            yaw_sp_calculated_m2p_data.yaw_sp);

}

/* publish messages to pixhawk. -libn */
state_machine::OBSTACLE_POSITION_M2P obstacle_position_m2p_data;
state_machine::TASK_STATUS_MONITOR_M2P task_status_monitor_m2p_data;
state_machine::VISION_NUM_SCAN_M2P vision_num_scan_m2p_data;
state_machine::VISION_ONE_NUM_GET_M2P vision_one_num_get_m2p_data;
state_machine::TASK_STATUS_CHANGE_P2M task_status_change_p2m_data;
void task_status_change_p2m_cb(const state_machine::TASK_STATUS_CHANGE_P2M::ConstPtr& msg){
	task_status_change_p2m_data = *msg;
	ROS_INFO("subscribing task_status_change_p2m: %5.3f %d %d",
				task_status_change_p2m_data.spray_duration,
				task_status_change_p2m_data.task_status,
				task_status_change_p2m_data.loop_value);
    task_status_monitor_m2p_data.spray_duration = task_status_change_p2m_data.spray_duration;
}

std_msgs::Int32 vision_num_data;
void vision_num_cb(const std_msgs::Int32::ConstPtr& msg){
    vision_num_data = *msg;
    current_mission_num = vision_num_data.data;
    ROS_INFO("subscribing vision_num_data = %d", vision_num_data.data);
}

std_msgs::Int32 camera_switch_data;
ros::Publisher  camera_switch_pub;

//void vision_one_num_get_cal(void)
//{
//	vision_one_num_get_m2p_data.loop_value = loop;
////	vision_one_num_get_m2p_data.num = ;	//TODO: add vision_num_get topic!
//}
//void yaw_sp_cal(void)
//{
//	yaw_sp_calculated_m2p_data.yaw_sp = 0.6f;
//}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<state_machine::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);

    /* Velocity setpoint. -libn */
    ros::Publisher local_vel_pub = nh.advertise<geometry_msgs::TwistStamped>
                ("/mavros/setpoint_velocity/cmd_vel", 10);

    ros::ServiceClient arming_client = nh.serviceClient<state_machine::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<state_machine::SetMode>
            ("mavros/set_mode");

    // takeoff and land service
    // ros::ServiceClient takeoff_client = nh.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/takeoff");
    ros::ServiceClient land_client = nh.serviceClient<state_machine::CommandTOL>("mavros/cmd/land");
    state_machine::CommandTOL landing_cmd;
    landing_cmd.request.min_pitch = 1.0;

	/* receive indexed setpoint. -libn */
	ros::Subscriber setpoint_Indexed_sub = nh.subscribe("Setpoint_Indexed", 100 ,SetpointIndexedCallback);
	/* get pixhawk's local position. -libn */
	ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, pos_cb);

    /* get pixhawk's local velocity. -libn */
    ros::Subscriber local_vel_sub = nh.subscribe<geometry_msgs::TwistStamped>("mavros/local_position/velocity", 10, vel_cb);

	ros::Subscriber DrawingBoard_Position_sub = nh.subscribe<state_machine::DrawingBoard10>
		            ("DrawingBoard_Position10", 10, board_pos_cb);
	board10.drawingboard.resize(10);		/* MUST! -libn */

	/* subscribe messages from pixhawk. -libn */
    ros::Subscriber fixed_target_position_p2m_sub = nh.subscribe<state_machine::FIXED_TARGET_POSITION_P2M>("mavros/fixed_target_position_p2m", 10, fixed_target_position_p2m_cb);
    ros::Subscriber task_status_change_p2m_sub = nh.subscribe<state_machine::TASK_STATUS_CHANGE_P2M>("mavros/task_status_change_p2m", 10, task_status_change_p2m_cb);

    /* publish messages to pixhawk. -libn */
    fixed_target_return_m2p_pub  = nh.advertise<state_machine::FIXED_TARGET_RETURN_M2P>("mavros/fixed_target_return_m2p", 10);
    ros::Publisher  obstacle_position_m2p_pub  = nh.advertise<state_machine::OBSTACLE_POSITION_M2P>("mavros/obstacle_position_m2p", 10);
    ros::Publisher  task_status_monitor_m2p_pub  = nh.advertise<state_machine::TASK_STATUS_MONITOR_M2P>("mavros/task_status_monitor_m2p", 10);
    ros::Publisher  vision_num_scan_m2p_pub  = nh.advertise<state_machine::VISION_NUM_SCAN_M2P>("mavros/vision_num_scan_m2p", 10);
    ros::Publisher  vision_one_num_get_m2p_pub  = nh.advertise<state_machine::VISION_ONE_NUM_GET_M2P>("mavros/vision_one_num_get_m2p", 10);
    yaw_sp_calculated_m2p_pub  = nh.advertise<state_machine::YAW_SP_CALCULATED_M2P>("mavros/yaw_sp_calculated_m2p", 10);

    camera_switch_pub  = nh.advertise<std_msgs::Int32>("camera_switch", 10);

    /*  camera_switch: 0: mission closed; 1: vision_one_num_get; 2: vision_num_scan. -libn */
    camera_switch_data.data = 0;

    /* get vision_num */
    ros::Subscriber vision_num_sub = nh.subscribe<std_msgs::Int32>("vision_num", 10, vision_num_cb);

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(10.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }


    if(1)   /* initialisation: send 100 setpoints. */
    {
        /* local velocity setpoint publish. -libn */
        vel_pub.twist.linear.x = 0.0f;
        vel_pub.twist.linear.y = 0.0f;
        vel_pub.twist.linear.z = 2.0f;
        vel_pub.twist.angular.x = 0.0f;
        vel_pub.twist.angular.y = 0.0f;
        vel_pub.twist.angular.z = 0.0f;
        ROS_INFO("sending 100 setpoints, please wait 10 seconds!");
        //send a few setpoints before starting
        for(int i = 100; ros::ok() && i > 0; --i){
//            local_pos_pub.publish(pose_pub);
            local_vel_pub.publish(vel_pub);
            ros::spinOnce();
            rate.sleep();
        }
    }


    ros::Time last_request = ros::Time::now();
    ros::Time last_show_request = ros::Time::now();

    last_state_display = current_state;
    last_state.mode = current_state.mode;
    last_state.armed = current_state.armed;
    ROS_INFO("current_state.mode = %s",current_state.mode.c_str());
    ROS_INFO("armed status: %d",current_state.armed);

    /* initialisation(loop once). */
    /* initialize: 4 fixed setpoints(A,L,R,D), yaw*, 10 board position,
     * current_mission_num, camera_switch_data. */
    if(1)
    {
        setpoint_H.pose.position.x = current_pos.pose.position.x;
        setpoint_H.pose.position.y = current_pos.pose.position.y;
        setpoint_H.pose.position.z = FIXED_POS_HEIGHT;  /* it's better to choose z* = SAFE_HEIGHT_DISTANCE(no altitude lost). */

        setpoint_A.pose.position.x = 0.0f;
        setpoint_A.pose.position.y = 0.0f;
        setpoint_A.pose.position.z = FIXED_POS_HEIGHT;

        setpoint_L.pose.position.x = 0.0f;
        setpoint_L.pose.position.y = 0.0f;
        setpoint_L.pose.position.z = FIXED_POS_HEIGHT;

        setpoint_R.pose.position.x = 0.0f;
        setpoint_R.pose.position.y = 0.0f;
        setpoint_R.pose.position.z = FIXED_POS_HEIGHT;

        setpoint_D.pose.position.x = 0.0f;
        setpoint_D.pose.position.y = 0.0f;
        setpoint_D.pose.position.z = FIXED_POS_HEIGHT;

        yaw_sp_calculated_m2p_data.yaw_sp = 90*M_PI/180;   /* default yaw*(90 degree)(ENU) -> North! */
        /* publish yaw_sp to pixhawk. */
        yaw_sp_calculated_m2p_pub.publish(yaw_sp_calculated_m2p_data);
        ROS_INFO("publishing yaw_sp_calculated_m2p: %f",
                yaw_sp_calculated_m2p_data.yaw_sp);

        /* yaw* for controller. */
        pose_pub.pose.orientation.x = 0;			/* orientation expressed using quaternion. -libn */
        pose_pub.pose.orientation.y = 0;			/* w = cos(theta/2), x = nx * sin(theta/2),  y = ny * sin(theta/2), z = nz * sin(theta/2) -libn */
        pose_pub.pose.orientation.z = sin(yaw_sp_calculated_m2p_data.yaw_sp/2);
        pose_pub.pose.orientation.w = cos(yaw_sp_calculated_m2p_data.yaw_sp/2);

        for(int co = 0; co<10; ++co)
        {
            /* set param valid true as default to make state machine able to run, and I am sure all numbers will be valid!(vision) */
            board10.drawingboard[co].valid = true;  /* Normly it should be false as the default setting. */
            board10.drawingboard[co].x = 0.0f;
            board10.drawingboard[co].y = 0.0f;
            board10.drawingboard[co].z = 0.0f;  /* it's safe for we have SAFE_HEIGHT_DISTANCE. */
        }
        current_mission_num = 0;    /* set current_mission_num as 0 as default. */
        last_mission_num = 0;

        /* camera_switch: 0: mission closed; 1: vision_one_num_get; 2: vision_num_scan. -libn */
        camera_switch_data.data = 0;    /* vision used for vision_num_scan as default. */
        camera_switch_pub.publish(camera_switch_data);
//        ROS_INFO("send camera_switch_data = %d",(int)camera_switch_data.data);

        /* default spray_duration */
        task_status_monitor_m2p_data.spray_duration = 1.0f;

        /* failure recorded. */
        failure[0].num = -1; failure[0].state = takeoff;
        failure[1].num = -1; failure[1].state = takeoff;
        failure[2].num = -1; failure[2].state = takeoff;
        failure[3].num = -1; failure[3].state = takeoff;
        failure[4].num = -1; failure[4].state = takeoff;

    }

    int send_vision_num_count = 0;  /* used to publish vision_scanning results. */

    while(ros::ok())
    {

        /* camera switch for test(set in state_machine_func for mission) and mode switch display(Once when freshed). -libn */
        if(1)
        {

            if(current_state.mode == "MANUAL" && last_state.mode != "MANUAL")
            {
                last_state.mode = "MANUAL";
                ROS_INFO("switch to mode: MANUAL");
                /* start manual scanning. -libn */
                /*  camera_switch/camera_switch_return: 0: mission closed; 1: vision_one_num_get; 2: vision_num_scan. -libn */
                camera_switch_data.data = 1;
                camera_switch_pub.publish(camera_switch_data);
                ROS_INFO("send camera_switch_data = %d",(int)camera_switch_data.data);

            }
            if(current_state.mode == "ALTCTL" && last_state.mode != "ALTCTL")
            {
                last_state.mode = "ALTCTL";
                ROS_INFO("switch to mode: ALTCTL");
                /* start manual scanning. -libn */
                /*  camera_switch/camera_switch_return: 0: mission closed; 1: vision_one_num_get; 2: vision_num_scan. -libn */
                camera_switch_data.data = 2;
                camera_switch_pub.publish(camera_switch_data);
                ROS_INFO("send camera_switch_data = %d",(int)camera_switch_data.data);
            }
            if(current_state.mode == "OFFBOARD" && last_state.mode != "OFFBOARD")
            {
                last_state.mode = "OFFBOARD";
                ROS_INFO("switch to mode: OFFBOARD");

            }
            if(current_state.armed && !last_state.armed)
                    {

                last_state.armed = current_state.armed;
                ROS_INFO("UAV armed!");
            }

        }
        if(current_state.mode == "MANUAL" && current_state.armed)
        {
            camera_switch_data.data = 0;    /* disable camere. */
            camera_switch_pub.publish(camera_switch_data);
            ROS_INFO("send camera_switch_data = %d",(int)camera_switch_data.data);
            current_mission_num = -1;    /* set current_mission_num as 0 as default. */
            last_mission_num = -1;   /* disable the initial mission_num gotten before takeoff. */

        }

        // landing
		if(current_state.armed && current_mission_state == land)	/* set landing mode until uav stops. -libn */
		{
			if(current_state.mode != "AUTO.LAND" && (ros::Time::now() - last_request > ros::Duration(5.0)))
			{
				if(land_client.call(landing_cmd) && landing_cmd.response.success)
				{
					ROS_INFO("AUTO LANDING!");
				}
				last_request = ros::Time::now();
			}
		}

        /* state_machine start and mission state display. -libn */
		if(current_state.mode == "OFFBOARD" && current_state.armed)	/* set message display delay(0.5s). -libn */
		{
			ROS_INFO("now I am in OFFBOARD and armed mode!");	/* state machine! -libn */

			state_machine_func();

            /* system timer. TODO! */
            if(1)
            {
                /* set time deadline. */
                /* mission timer(5 loops). -libn */
                if(ros::Time::now() - mission_timer_start_time > ros::Duration(MAX_FLIGHT_TIME)
                        && force_home_enable == true)
                {
                    force_home_enable = false; /* force once! */
                    current_mission_state = mission_force_return_home;	/* mission timeout. -libn */
                    ROS_INFO("mission time out! -> return to home!");
                    mission_last_time = ros::Time::now();   /* start counting time(for hovering). */
                }
                /* subtask timer(1 loop). -libn */
                if(ros::Time::now() - mission_timer_start_time > ros::Duration((float)(loop*30.0f+50.0f)) &&
                   ros::Time::now() - mission_timer_start_time < ros::Duration(MAX_FLIGHT_TIME) &&
                        loop <= 5)  /* stop subtask timer when dealing with failures. */
                {
                    /* error recorded! */
                    mission_failure_acount++;
                    failure[mission_failure_acount-1].num = current_mission_num;
                    failure[mission_failure_acount-1].state = current_mission_state;
                    loop++;
                    ROS_INFO("loop timeout -> start next loop");
                    current_mission_state = mission_observe_point_go;	/* loop timeout, forced to switch to next loop. -libn */
                    /* TODO: mission failure recorded(using switch/case). -libn */

                }
            }

            if(1)   /* ROS_INFO display. */
            {
                ROS_INFO("current loop: %d",loop);
                ROS_INFO("current_mission_state: %d",current_mission_state);
                if(velocity_control_enable)
                {
                    ROS_INFO("velocity*: %5.3f %5.3f %5.3f",vel_pub.twist.linear.x, vel_pub.twist.linear.y, vel_pub.twist.linear.z);
                }
                else
                {
                    ROS_INFO("position*: %5.3f %5.3f %5.3f",pose_pub.pose.position.x,pose_pub.pose.position.y,pose_pub.pose.position.z);
                }

                ROS_INFO("current position: %5.3f %5.3f %5.3f\n",current_pos.pose.position.x,current_pos.pose.position.y,current_pos.pose.position.z);

                ROS_INFO("current_mission_num = %d",current_mission_num);
                ROS_INFO("board: current_mission_num: %d\n"
                        "position:%5.3f %5.3f %5.3f",current_mission_num,board10.drawingboard[current_mission_num].x,
                        board10.drawingboard[current_mission_num].y,board10.drawingboard[current_mission_num].z);




    //			ROS_INFO("board_position_received:\n"
    //					"board0: %d %5.3f %5.3f %5.3f \n"
    //					"board1: %d %5.3f %5.3f %5.3f \n"
    //					"board2: %d %5.3f %5.3f %5.3f \n"
    //					"board3: %d %5.3f %5.3f %5.3f \n"
    //					"board4: %d %5.3f %5.3f %5.3f \n"
    //					"board5: %d %5.3f %5.3f %5.3f \n"
    //					"board6: %d %5.3f %5.3f %5.3f \n"
    //					"board7: %d %5.3f %5.3f %5.3f \n"
    //					"board8: %d %5.3f %5.3f %5.3f \n"
    //					"board9: %d %5.3f %5.3f %5.3f \n",
    //					board10.drawingboard[0].valid,board10.drawingboard[0].x,board10.drawingboard[0].y,board10.drawingboard[0].z,
    //					board10.drawingboard[1].valid,board10.drawingboard[1].x,board10.drawingboard[1].y,board10.drawingboard[1].z,
    //					board10.drawingboard[2].valid,board10.drawingboard[2].x,board10.drawingboard[2].y,board10.drawingboard[2].z,
    //					board10.drawingboard[3].valid,board10.drawingboard[3].x,board10.drawingboard[3].y,board10.drawingboard[3].z,
    //					board10.drawingboard[4].valid,board10.drawingboard[4].x,board10.drawingboard[4].y,board10.drawingboard[4].z,
    //					board10.drawingboard[5].valid,board10.drawingboard[5].x,board10.drawingboard[5].y,board10.drawingboard[5].z,
    //					board10.drawingboard[6].valid,board10.drawingboard[6].x,board10.drawingboard[6].y,board10.drawingboard[6].z,
    //					board10.drawingboard[7].valid,board10.drawingboard[7].x,board10.drawingboard[7].y,board10.drawingboard[7].z,
    //					board10.drawingboard[8].valid,board10.drawingboard[8].x,board10.drawingboard[8].y,board10.drawingboard[8].z,
    //					board10.drawingboard[9].valid,board10.drawingboard[9].x,board10.drawingboard[9].y,board10.drawingboard[9].z);

            }

		}
        else    /* All information display. */
		{
			if(current_state.mode != last_state_display.mode || last_state_display.armed != current_state.armed)
			{
				ROS_INFO("current_state.mode = %s",current_state.mode.c_str());
				ROS_INFO("last_state_display.mode = %s",last_state_display.mode.c_str());
				ROS_INFO("armed status: %d\n",current_state.armed);
				last_state_display.armed = current_state.armed;
				last_state_display.mode = current_state.mode;
				ROS_INFO("current position: %5.3f %5.3f %5.3f", current_pos.pose.position.x, 	  	current_pos.pose.position.y, current_pos.pose.position.z);

				ROS_INFO("setpoint_received:\n"
                        "setpoint_A(ENU):%5.3f %5.3f %5.3f \n"
                        "setpoint_L(ENU):%5.3f %5.3f %5.3f \n"
                        "setpoint_R(ENU):%5.3f %5.3f %5.3f \n"
                        "setpoint_D(ENU):%5.3f %5.3f %5.3f \n"
                        "setpoint_H(ENU):%5.3f %5.3f %5.3f",
						setpoint_A.pose.position.x,setpoint_A.pose.position.y,setpoint_A.pose.position.z,
                        setpoint_L.pose.position.x,setpoint_L.pose.position.y,setpoint_L.pose.position.z,
                        setpoint_R.pose.position.x,setpoint_R.pose.position.y,setpoint_R.pose.position.z,
						setpoint_D.pose.position.x,setpoint_D.pose.position.y,setpoint_D.pose.position.z,
                        setpoint_H.pose.position.x,setpoint_H.pose.position.y,setpoint_H.pose.position.z);
                ROS_INFO("yaw_sp(ENU) = rad:%f deg:%f",yaw_sp_calculated_m2p_data.yaw_sp,yaw_sp_calculated_m2p_data.yaw_sp*180/M_PI);
                ROS_INFO("board_position_received(ENU):\n"
						"board0: %d %5.3f %5.3f %5.3f \n"
						"board1: %d %5.3f %5.3f %5.3f \n"
						"board2: %d %5.3f %5.3f %5.3f \n"
						"board3: %d %5.3f %5.3f %5.3f \n"
						"board4: %d %5.3f %5.3f %5.3f \n"
						"board5: %d %5.3f %5.3f %5.3f \n"
						"board6: %d %5.3f %5.3f %5.3f \n"
						"board7: %d %5.3f %5.3f %5.3f \n"
						"board8: %d %5.3f %5.3f %5.3f \n"
						"board9: %d %5.3f %5.3f %5.3f \n",
						board10.drawingboard[0].valid,board10.drawingboard[0].x,board10.drawingboard[0].y,board10.drawingboard[0].z,
						board10.drawingboard[1].valid,board10.drawingboard[1].x,board10.drawingboard[1].y,board10.drawingboard[1].z,
						board10.drawingboard[2].valid,board10.drawingboard[2].x,board10.drawingboard[2].y,board10.drawingboard[2].z,
						board10.drawingboard[3].valid,board10.drawingboard[3].x,board10.drawingboard[3].y,board10.drawingboard[3].z,
						board10.drawingboard[4].valid,board10.drawingboard[4].x,board10.drawingboard[4].y,board10.drawingboard[4].z,
						board10.drawingboard[5].valid,board10.drawingboard[5].x,board10.drawingboard[5].y,board10.drawingboard[5].z,
						board10.drawingboard[6].valid,board10.drawingboard[6].x,board10.drawingboard[6].y,board10.drawingboard[6].z,
						board10.drawingboard[7].valid,board10.drawingboard[7].x,board10.drawingboard[7].y,board10.drawingboard[7].z,
						board10.drawingboard[8].valid,board10.drawingboard[8].x,board10.drawingboard[8].y,board10.drawingboard[8].z,
						board10.drawingboard[9].valid,board10.drawingboard[9].x,board10.drawingboard[9].y,board10.drawingboard[9].z);
                ROS_INFO("current_mission_num = %d",current_mission_num);
                ROS_INFO("board: current_mission_num: %d\n"
                        "position:%5.3f %5.3f %5.3f",current_mission_num,board10.drawingboard[current_mission_num].x,
                        board10.drawingboard[current_mission_num].y,board10.drawingboard[current_mission_num].z);
//                ROS_INFO("SCREEN_HEIGHT = %d SAFE_HEIGHT_DISTANCE = %d",(int)SCREEN_HEIGHT,(int)SAFE_HEIGHT_DISTANCE);
                ROS_INFO("SAFE_HEIGHT_DISTANCE = %d",(int)SAFE_HEIGHT_DISTANCE);

			}
		}

        if(!velocity_control_enable)    /* position control. */
        {
            /* limit error(x,y) between current position and destination within [-1,1]. */
            if(abs(pose_pub.pose.position.x - current_pos.pose.position.x) > 30 ||
                abs(pose_pub.pose.position.y - current_pos.pose.position.y) > 30)
            {
                double error_temp[2] = {0,0};
                error_limit(current_pos.pose.position.x,current_pos.pose.position.y,pose_pub.pose.position.x,pose_pub.pose.position.y,error_temp);
                pose_pub.pose.position.x = current_pos.pose.position.x + 30*error_temp[0];
                pose_pub.pose.position.y = current_pos.pose.position.y + 30*error_temp[1];
            }
        }

        if(1)   /* publish messages to pixhawk. */
        {
            /* publish messages to pixhawk. -libn */
//            obstacle_position_m2p_data.obstacle_x = 2.0f;
//            obstacle_position_m2p_data.obstacle_y = 2.0f;
//            obstacle_position_m2p_data.obstacle_z = 2.0f;
//            obstacle_position_m2p_data.obstacle_valid = true;
//            obstacle_position_m2p_pub.publish(obstacle_position_m2p_data);
//    //		ROS_INFO("publishing obstacle_position_m2p: %f\t%f\t%f\t%d",
//    //				obstacle_position_m2p_data.obstacle_x,
//    //				obstacle_position_m2p_data.obstacle_y,
//    //				obstacle_position_m2p_data.obstacle_z,
//    //				obstacle_position_m2p_data.obstacle_valid);

//            task_status_monitor_m2p_data.spray_duration = 0.3f;
            task_status_monitor_m2p_data.task_status = current_mission_state;
            task_status_monitor_m2p_data.loop_value = loop;
            if(velocity_control_enable)
            {
                task_status_monitor_m2p_data.target_x = current_pos.pose.position.y;
                task_status_monitor_m2p_data.target_y = current_pos.pose.position.x;
                task_status_monitor_m2p_data.target_z = -current_pos.pose.position.z;
            }
            else
            {
                task_status_monitor_m2p_data.target_x = pose_pub.pose.position.y;
                task_status_monitor_m2p_data.target_y = pose_pub.pose.position.x;
                task_status_monitor_m2p_data.target_z = -pose_pub.pose.position.z;
            }
            task_status_monitor_m2p_pub.publish(task_status_monitor_m2p_data);
    //		ROS_INFO("publishing task_status_monitor_m2p: %f %d %d %f %f %f",
    //				task_status_monitor_m2p_data.spray_duration,
    //				task_status_monitor_m2p_data.task_status,
    //				task_status_monitor_m2p_data.loop_value,
    //				task_status_monitor_m2p_data.target_lat,
    //				task_status_monitor_m2p_data.target_lon,
    //				task_status_monitor_m2p_data.target_alt);

//            for(send_vision_num_count = 0;send_vision_num_count<10;send_vision_num_count++)
//            {
            send_vision_num_count++;
            send_vision_num_count = send_vision_num_count % 10;
            vision_num_scan_m2p_data.board_num = send_vision_num_count;
            vision_num_scan_m2p_data.board_x = board10.drawingboard[send_vision_num_count].y,
            vision_num_scan_m2p_data.board_y = board10.drawingboard[send_vision_num_count].x,
            vision_num_scan_m2p_data.board_z = -board10.drawingboard[send_vision_num_count].z,
            vision_num_scan_m2p_data.board_valid = board10.drawingboard[send_vision_num_count].valid;

            vision_num_scan_m2p_pub.publish(vision_num_scan_m2p_data);
//            }
//            send_vision_num_count = send_vision_num_count % 10;

    //		ROS_INFO("publishing vision_num_scan_m2p: %d %f %f %f %d",
    //				vision_num_scan_m2p_data.board_num,
    //				vision_num_scan_m2p_data.board_x,
    //				vision_num_scan_m2p_data.board_y,
    //				vision_num_scan_m2p_data.board_z,
    //				vision_num_scan_m2p_data.board_valid);

            vision_one_num_get_m2p_data.loop_value = loop;
            vision_one_num_get_m2p_data.num = current_mission_num;
            vision_one_num_get_m2p_pub.publish(vision_one_num_get_m2p_data);
    //		ROS_INFO("publishing vision_one_num_get_m2p: %d %d",
    //				vision_one_num_get_m2p_data.loop_value,
    //				vision_one_num_get_m2p_data.num);

        }


        if(velocity_control_enable)
        {
        	local_vel_pub.publish(vel_pub);
        }
        else
        {
        	local_pos_pub.publish(pose_pub);
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

/* task state machine. -libn */
void state_machine_func(void)
{
	switch(current_mission_state)
	{
		case takeoff:
            /* start mission_timer once. */
            if(mission_timer_enable)
            {
                mission_timer_start_time = ros::Time::now();    /* start mission_timer. */
                mission_timer_enable = false;
            }
			/* local velocity setpoint publish. -libn */
            velocity_control_enable = true;
            vel_pub.twist.linear.x = 0.0f;
			vel_pub.twist.linear.y = 0.0f;
            vel_pub.twist.linear.z = 2.0f;
			vel_pub.twist.angular.x = 0.0f;
			vel_pub.twist.angular.y = 0.0f;
			vel_pub.twist.angular.z = 0.0f;
            if(current_vel.twist.linear.z > 0.5 &&
               current_pos.pose.position.z > 0.8)
            {
                ROS_INFO("current_vel.twist.linear.x = %f",current_vel.twist.linear.x);

                current_mission_state = mission_hover_after_takeoff; // current_mission_state++;
                mission_last_time = ros::Time::now();

                velocity_control_enable = false;
                pose_pub.pose.position.x = current_pos.pose.position.x;
                pose_pub.pose.position.y = current_pos.pose.position.y;
                pose_pub.pose.position.z = setpoint_H.pose.position.z;

                /*  camera_switch: 0: mission closed; 1: vision_one_num_get; 2: vision_num_scan. -libn */
                camera_switch_data.data = 0;
                camera_switch_pub.publish(camera_switch_data);
                ROS_INFO("send camera_switch_data = %d",(int)camera_switch_data.data);

            }
    		break;

        case mission_hover_after_takeoff:
        	pose_pub.pose.position.x = current_pos.pose.position.x;
        	pose_pub.pose.position.y = current_pos.pose.position.y;
        	pose_pub.pose.position.z = setpoint_H.pose.position.z;
            if(ros::Time::now() - mission_last_time > ros::Duration(3))	/* hover for 5 seconds. -libn */
        	{
                current_mission_state = mission_observe_point_go; // current_mission_state++;
        	}
            break;
        case mission_observe_point_go:
        	if(loop > 5)
			{
                current_mission_state = mission_num_done; // current_mission_state++;
				break;
			}
        	pose_pub.pose.position.x = setpoint_A.pose.position.x;
			pose_pub.pose.position.y = setpoint_A.pose.position.y;
            pose_pub.pose.position.z = setpoint_A.pose.position.z;
            /* camera_switch revised. */
            if((abs(current_pos.pose.position.x - pose_pub.pose.position.x) < 1.0) &&      // switch to next state
               (abs(current_pos.pose.position.y - pose_pub.pose.position.y) < 1.0))
            {
                /* camera_switch: 0: mission closed; 1: vision_one_num_get; 2: vision_num_scan. -libn */
                camera_switch_data.data = 1;
                camera_switch_pub.publish(camera_switch_data);
                ROS_INFO("send camera_switch_data = %d",(int)camera_switch_data.data);
            }
            if((abs(current_pos.pose.position.x - pose_pub.pose.position.x) < 0.2) &&      // switch to next state
               (abs(current_pos.pose.position.y - pose_pub.pose.position.y) < 0.2) &&
               (abs(current_pos.pose.position.z - pose_pub.pose.position.z) < 0.2))
            {
                current_mission_state = mission_observe_num_wait; // current_mission_state++;
            	mission_last_time = ros::Time::now();
            }
            break;
        case mission_observe_num_wait:
        	pose_pub.pose.position.x = setpoint_A.pose.position.x;
			pose_pub.pose.position.y = setpoint_A.pose.position.y;
			pose_pub.pose.position.z = setpoint_A.pose.position.z;
//			if(!display_screen_num_recognized)
//			{
//				/* TODO: to recognize the number. -libn */
//				display_screen_num_recognized = true;	/* number recognized. -libn */
//
//
//				if(loop == 1 && (ros::Time::now() - mission_last_time > ros::Duration(50)))	/* wait 50 seconds at most for the first time. -libn */
//				{
//					current_mission_state = mission_num_search; // current_mission_state++;
//					/* TODO: failure recorded. -libn */
//				}
//				else
//				{
//					if(loop!= 1 && ros::Time::now() - mission_last_time > ros::Duration(10))	/* wait 10 seconds at most for recognition. -libn */
//					{
//						current_mission_state = mission_num_search; // current_mission_state++;
//						/* TODO: failure recorded. -libn */
//					}
//				}
//			}
//			else
//			{
//				current_mission_state = mission_num_search; // current_mission_state++;
//			}
			//time delay added(just for test! --delete it directly!)
            if(current_mission_num == last_mission_num)
            {
                current_mission_state = mission_observe_num_wait; // stop mission_state++, avoid repeating spraying. ;
            }
            else    /* valid number detected. */
            {
                if(ros::Time::now() - mission_last_time > ros::Duration(1))	/* hover for 1 seconds. -libn */
                {
                    current_mission_state = mission_num_search; // current_mission_state++;

                    /* change and publish camera_switch_data for next subtask. */
                    /*  camera_switch: 0: mission closed; 1: vision_one_num_get; 2: vision_num_scan. -libn */
                    camera_switch_data.data = 2;
                    camera_switch_pub.publish(camera_switch_data);
                    ROS_INFO("send camera_switch_data = %d",(int)camera_switch_data.data);

                    last_mission_num = current_mission_num;
                }
            }

            break;
        case mission_num_search:
//        	ROS_INFO("board10.drawingboard[current_mission_num].valid = %d",board10.drawingboard[current_mission_num].valid);
        	if(board10.drawingboard[current_mission_num].valid)
        	{
//        		ROS_INFO("Now I am making decision.\n"
//        				"current_pos: x = %5.3f y = %5.3f z = %5.3f\n"
//        				"board10.drawingboard[%d]_pos: x = %5.3f y = %5.3f z = %5.3f\n",
//						current_pos.pose.position.x, current_pos.pose.position.y, current_pos.pose.position.z,
//						current_mission_num,board10.drawingboard[current_mission_num].x,
//						board10.drawingboard[current_mission_num].y,board10.drawingboard[current_mission_num].z);
//
//        		ROS_INFO("position*: %5.3f %5.3f %5.3f",pose_pub.pose.position.x,pose_pub.pose.position.y,pose_pub.pose.position.z);
//				ROS_INFO("current position: %5.3f %5.3f %5.3f",current_pos.pose.position.x,current_pos.pose.position.y,current_pos.pose.position.z);

                pose_pub.pose.position.x = board10.drawingboard[current_mission_num].x - VISION_SCAN_DISTANCE * cos(yaw_sp_calculated_m2p_data.yaw_sp);	/* TODO:switch to different board positions. -libn */
                pose_pub.pose.position.y = board10.drawingboard[current_mission_num].y - VISION_SCAN_DISTANCE * sin(yaw_sp_calculated_m2p_data.yaw_sp);
                pose_pub.pose.position.z = board10.drawingboard[current_mission_num].z + SAFE_HEIGHT_DISTANCE;

//				ROS_INFO("distance: x = %5.3f y = %5.3f z = %5.3f\n",
//						abs(current_pos.pose.position.x - board10.drawingboard[current_mission_num].x),
//						abs(current_pos.pose.position.y - board10.drawingboard[current_mission_num].y),
//						abs(current_pos.pose.position.z - (board10.drawingboard[current_mission_num].z+3)));

                if((abs(current_pos.pose.position.x - pose_pub.pose.position.x) < 0.2) &&      // switch to next state
                   (abs(current_pos.pose.position.y - pose_pub.pose.position.y) < 0.2) &&
                   (abs(current_pos.pose.position.z - pose_pub.pose.position.z) < 0.2))
				{
                    current_mission_state = mission_num_locate; // current_mission_state++;
					mission_last_time = ros::Time::now();
					ROS_INFO("mission switched well!");
				}
        	}
        	else
        	{
        		/* TODO: add scanning method! -libn */
                current_mission_state = mission_num_scan_again; // current_mission_state++;
                ROS_INFO("fall into mission state: mission_num_scan_again");
        	}
			break;
        case mission_num_locate:
            pose_pub.pose.position.x = board10.drawingboard[current_mission_num].x - VISION_SCAN_DISTANCE * cos(yaw_sp_calculated_m2p_data.yaw_sp);	/* TODO:switch to different board positions. -libn */
            pose_pub.pose.position.y = board10.drawingboard[current_mission_num].y - VISION_SCAN_DISTANCE * sin(yaw_sp_calculated_m2p_data.yaw_sp);
            pose_pub.pose.position.z = board10.drawingboard[current_mission_num].z + SAFE_HEIGHT_DISTANCE;
            /* TODO:update the position of the drawing board.  -libn */
			relocate_valid = true;

			if(relocate_valid)
			{
                if(ros::Time::now() - mission_last_time > ros::Duration(5))	/* hover for 5 seconds. -libn */
                {
                    current_mission_state = mission_num_get_close; // current_mission_state++;
                    mission_last_time = ros::Time::now();
                }
			}
			else
			{
                if(ros::Time::now() - mission_last_time > ros::Duration(5))	/* wait for this operate 5 seconds at most. -libn */
				{
					/* TODO: add scanning method! -libn */
                    current_mission_state = mission_num_scan_again; // current_mission_state++;
				}
			}            

			break;
        case mission_num_get_close:
            pose_pub.pose.position.x = board10.drawingboard[current_mission_num].x - SPRAY_DISTANCE * cos(yaw_sp_calculated_m2p_data.yaw_sp);	/* TODO:switch to different board positions. -libn */
            pose_pub.pose.position.y = board10.drawingboard[current_mission_num].y - SPRAY_DISTANCE * sin(yaw_sp_calculated_m2p_data.yaw_sp);
            pose_pub.pose.position.z = board10.drawingboard[current_mission_num].z + SAFE_HEIGHT_DISTANCE;
            if((abs(current_pos.pose.position.x - pose_pub.pose.position.x) < 0.2) &&      // switch to next state
               (abs(current_pos.pose.position.y - pose_pub.pose.position.y) < 0.2) &&
               (abs(current_pos.pose.position.z - pose_pub.pose.position.z) < 0.2))
               {
                current_mission_state = mission_hover_before_spary; // current_mission_state++;
            	mission_last_time = ros::Time::now();
               }
            break;
        case mission_hover_before_spary:
            pose_pub.pose.position.x = board10.drawingboard[current_mission_num].x - SPRAY_DISTANCE * cos(yaw_sp_calculated_m2p_data.yaw_sp);	/* TODO:switch to different board positions. -libn */
            pose_pub.pose.position.y = board10.drawingboard[current_mission_num].y - SPRAY_DISTANCE * sin(yaw_sp_calculated_m2p_data.yaw_sp);
            pose_pub.pose.position.z = board10.drawingboard[current_mission_num].z + SAFE_HEIGHT_DISTANCE;
            if(ros::Time::now() - mission_last_time > ros::Duration(3))	/* hover for 5 seconds. -libn */
            {
                current_mission_state = mission_arm_spread; // current_mission_state++;
                mission_last_time = ros::Time::now();
                /* TODO: start spraying. -libn */

            }
            break;
        case mission_arm_spread:
            pose_pub.pose.position.x = board10.drawingboard[current_mission_num].x - SPRAY_DISTANCE * cos(yaw_sp_calculated_m2p_data.yaw_sp);	/* TODO:switch to different board positions. -libn */
            pose_pub.pose.position.y = board10.drawingboard[current_mission_num].y - SPRAY_DISTANCE * sin(yaw_sp_calculated_m2p_data.yaw_sp);
            pose_pub.pose.position.z = board10.drawingboard[current_mission_num].z + SAFE_HEIGHT_DISTANCE;
            if(ros::Time::now() - mission_last_time > ros::Duration(2))	/* hover for 5 seconds. -libn */
        	{
                current_mission_state = mission_num_hover_spray; // current_mission_state++;
        		mission_last_time = ros::Time::now();
        		/* TODO: start spraying. -libn */

        	}
            break;
        case mission_num_hover_spray:
            pose_pub.pose.position.x = board10.drawingboard[current_mission_num].x - SPRAY_DISTANCE * cos(yaw_sp_calculated_m2p_data.yaw_sp);	/* TODO:switch to different board positions. -libn */
            pose_pub.pose.position.y = board10.drawingboard[current_mission_num].y - SPRAY_DISTANCE * sin(yaw_sp_calculated_m2p_data.yaw_sp);
            pose_pub.pose.position.z = board10.drawingboard[current_mission_num].z + SAFE_HEIGHT_DISTANCE;
            if(ros::Time::now() - mission_last_time > ros::Duration(5))	/* spray for 5 seconds. -libn */
            {
                current_mission_state = mission_hover_after_stretch_back; // current_mission_state++;
                mission_last_time = ros::Time::now();
            }
            break;
        case mission_hover_after_stretch_back:
            pose_pub.pose.position.x = current_pos.pose.position.x;	/* hover in current position. -libn */
            pose_pub.pose.position.y = current_pos.pose.position.y;
            pose_pub.pose.position.z = current_pos.pose.position.z;
            if(ros::Time::now() - mission_last_time > ros::Duration(0))	/* spray for 5 seconds. -libn */
            {
                loop++;	/* switch to next loop. -libn */
                if(loop > 5)
                {
                    current_mission_state = mission_num_done; // current_mission_state++;
                }
                else
                {
                    current_mission_state = mission_observe_point_go; // current_mission_state++;
                }
            }
            break;
        case mission_num_done:
        	pose_pub.pose.position.x = current_pos.pose.position.x;	/* hover in current position. -libn */
        	pose_pub.pose.position.y = current_pos.pose.position.y;
        	pose_pub.pose.position.z = current_pos.pose.position.z;
        	/* TODO: mission check: if there are failures to be fixed -libn */
            if(FAILURE_REPAIR && mission_failure_acount != 0)
        	{
        		current_mission_state = mission_fix_failure; // current_mission_state++;
        		ROS_INFO("TODO: mission_fix_failure!");
        	}
        	else			/* mission is finished. -libn */
        	{
                current_mission_state = mission_return_home; // current_mission_state++;
                ROS_INFO("going to mission_return_home");
        		mission_last_time = ros::Time::now();
        	}
			break;
        case mission_fix_failure:
            /* TODO: add mission_failure_acount_fixed. -libn */
            if((ros::Time::now() - mission_timer_start_time < ros::Duration(MAX_FLIGHT_TIME))
                    && (mission_failure_acount != 0))
            {
                current_mission_num = failure[mission_failure_acount-1].num;
                /* deal with failures. */
                if(mission_num_search < failure[mission_failure_acount-1].state < mission_arm_spread ||
                        failure[mission_failure_acount-1].state == mission_hover_before_spary)
                {
                    current_mission_state = mission_num_search;
                }
                mission_failure_acount--;

            }
            else if(mission_failure_acount == 0)
            {
                ROS_INFO("All failure fixed, return to home.");
                current_mission_state = mission_return_home;
            }
			break;
        case mission_force_return_home:
            pose_pub.pose.position.x = current_pos.pose.position.x;
            pose_pub.pose.position.y = current_pos.pose.position.y;
            pose_pub.pose.position.z = current_pos.pose.position.z;
            if(ros::Time::now() - mission_last_time > ros::Duration(2))	/* hover for 2 seconds. -libn */
            {
                current_mission_state = mission_return_home;    /* force to return to home! */
            }
            break;
        case mission_return_home:
			pose_pub.pose.position.x = setpoint_H.pose.position.x;
			pose_pub.pose.position.y = setpoint_H.pose.position.y;
			pose_pub.pose.position.z = setpoint_H.pose.position.z;
//			ROS_INFO("start mission_return_home");
//			ROS_INFO("setpoint_H*: %5.3f %5.3f %5.3f",setpoint_H.pose.position.x,setpoint_H.pose.position.y,setpoint_H.pose.position.z);
//			ROS_INFO("current position --2 : %5.3f %5.3f %5.3f",current_pos.pose.position.x,current_pos.pose.position.y,current_pos.pose.position.z);
			/* Bug! -libn */
			if((abs(current_pos.pose.position.x - setpoint_H.pose.position.x) < 0.2) &&      // switch to next state
			   (abs(current_pos.pose.position.y - setpoint_H.pose.position.y) < 0.2) &&
			   (abs(current_pos.pose.position.z - setpoint_H.pose.position.z) < 0.2) &&
               (ros::Time::now() - mission_last_time > ros::Duration(1)))		/* Bug: mission_last_time is not necessary! -libn */
			{
                ROS_INFO("start mission_hover_only");
                current_mission_state = mission_hover_only; // current_mission_state++;
				mission_last_time = ros::Time::now();
			}
			break;
        case mission_hover_only:
			pose_pub.pose.position.x = setpoint_H.pose.position.x;
			pose_pub.pose.position.y = setpoint_H.pose.position.y;
			pose_pub.pose.position.z = setpoint_H.pose.position.z;
            if(ros::Time::now() - mission_last_time > ros::Duration(5))	/* hover for 2 seconds. -libn */
			{
				current_mission_state = land; // current_mission_state++;
			}
			break;
        case land:
			break;
    }

}
