 /**
* @file     : offb_simulation_test.cpp
* @brief    : offboard simulation test: demo rewritten -> 4 setpoints flight -> complete state machine.
* @author   : libn
* @Time     : Aug 25, 2016, 10:06:42 PM
*/

#include <ros/ros.h>
#include <math.h>
#include <std_msgs/Int32.h>               /*  */

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>   /* local velocity setpoint. -libn */
#include <state_machine/CommandBool.h>
#include <state_machine/SetMode.h>
#include <state_machine/State.h>
#include <state_machine/CommandTOL.h>
#include <state_machine/Setpoint.h>       /* 虚拟航迹点 */
#include <state_machine/DrawingBoard10.h> /* 喷绘板数字、位置 */

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
#include <state_machine/FailureRecord.h>

// 根据该变量是否定义，后面会显示一些提示
// 并决定ROS循环的运行频率
/* on ros info msg */
#define NO_ROS_DEBUG
/* select ROS rate */
#ifdef NO_ROS_DEBUG
    #define ROS_RATE 20.0
#else
    #define ROS_RATE 10.0
#endif

// 宏定义
// 喷洒时，会向前飞0.5m
#define SPRAY_DISTANCE 2.2  /* distance from UAV to drawing board while sparying. */
#define VISION_SCAN_DISTANCE 2.7  /* distance from UAV to drawing board while hoveing and scanning. */
// 为喷洒高度进行一个补偿
#define SAFE_HEIGHT_DISTANCE 0.42  /* distanche from drawing board's height to expected height: 0: real mission; >0: for safe. */
// 固定的高度
#define FIXED_POS_HEIGHT 1    /* height of point: O (A). */
#define TAKEOFF_HEIGHT 1.8  /* height of point H. */
#define SCAN_HEIGHT 1 /* constant height while scanning. */ //height of point: L, R.
// 预扫时距离喷绘板的距离
#define SCAN_VISION_DISTANCE 4
// 标志位，表示是否处于喷绘阶段的预扫
bool scan_to_get_pos = false;
// 固定的预扫速度，没用到
#define SCAN_MOVE_SPEED 2 /* error bewteen pos* and pos. */
// 最大飞行时间
#define MAX_FLIGHT_TIME 540 /* max flight time of whole mission. */ //暂时没考虑超时修复的时间限制
#define FAILURE_REPAIR 1    /* FAILURE_REPAIR: 0: never repair errores;
                                               1: repair errors. */
// 每个子任务的执行周期，有一定的预留时间
#define SUB_PERIOD 55

// 任务状态机函数
void state_machine_func(void);
/* mission state. -libn */ //任务状态，每个任务状态都要考虑任务超时的情况
static const int takeoff = 1;
static const int mission_hover_after_takeoff = 2;
static const int mission_hover_only = 3;
static const int mission_observe_point_go = 5;
static const int mission_observe_num_wait = 6;    //可能可以修复
static const int mission_num_search = 8;          //可修复
static const int mission_num_scan_again = 9;      //可修复
static const int mission_num_locate = 10;         //可修复
static const int mission_num_get_close = 11;      //可修复
static const int mission_arm_spread = 12;         //可修复
static const int mission_num_hover_spray = 13;    //不可能发生故障
static const int mission_num_done = 14;
static const int mission_return_home = 15;
static const int land = 16;
static const int mission_end = 17;
static const int mission_hover_before_spary = 18;
static const int mission_fix_failure = 19;
static const int mission_hover_after_stretch_back = 20;
static const int mission_force_return_home = 21;
/* state added for scanning mission. */
// 为了一开始的预扫
static const int mission_scan_left_go = 31;
static const int mission_scan_right_move = 32;
static const int mission_scan_right_hover = 33;
static const int mission_scan_left_move = 34;
static const int mission_scan_left_hover = 35;
static const int mission_scan_left2_hover = 36;

// 下面两个变量配合表示任务状态
int loop = 0;	/* loop calculator: loop = 0/1/2/3/4/5. -libn */
int current_mission_state = takeoff; //current mission state, initial state is to takeoff
ros::Time mission_last_time;	/* timer used in mission. -libn */
bool display_screen_num_recognized = false;	/* to check if the num on display screen is recognized. -libn */ //没用
// 到达任务喷绘板前面，是不是做一个旋停
bool relocate_valid = false;	/* to complete relocate mission. -libn */
// 当前任务数字与下一个任务数字
int current_mission_num;	/* mission num: 5 subtask -> 5 current nums.	TODO:change mission num. -libn */
int last_mission_num;
// 速度控制使能，发布位置期望或速度期望的标志
bool velocity_control_enable = true;
// 一个控制任务时间的定时器
ros::Time mission_timer_start_time;	/* timer to control the whole mission and 5 subtasks. -libn */
bool mission_timer_enable = true;   /* start mission_timer. */ //用于一次开启定时器
// 强制回home点的标志（总任务超时）
bool force_home_enable = true;
// 子任务定时器
ros::Time current_subtask_time; //当前子任务的开始时间
bool subtimer_start = true; //子任务定时器的开启标志

// 4个固定的任务点
/* 4 setpoints. -libn */
geometry_msgs::PoseStamped setpoint_A;  //观察点，O点
geometry_msgs::PoseStamped setpoint_L;
geometry_msgs::PoseStamped setpoint_R;
geometry_msgs::PoseStamped setpoint_D;
geometry_msgs::PoseStamped setpoint_H;	/* home position. -libn */
// 由位置计算的航向期望
state_machine::YAW_SP_CALCULATED_M2P yaw_sp_calculated_m2p_data,yaw_sp_pub2GCS;

// 要发布的位置、速度期望
geometry_msgs::PoseStamped pose_pub;
geometry_msgs::TwistStamped vel_pub;	/* velocity setpoint to be published. -libn */

// 记录发生故障的数字与状态
state_machine::FailureRecord failure[5];
int mission_failure_acount = 0;

// 订阅飞行状态和相应的回调函数
state_machine::State current_state;
state_machine::State last_state;
state_machine::State last_state_display;
void state_cb(const state_machine::State::ConstPtr& msg){
	last_state_display.mode = current_state.mode;
	last_state_display.armed = current_state.armed;
	current_state = *msg;
}

// 两点见间的距离
/* calculate distance */
double circle_distance(double x1, double x2, double y1, double y2, double z1, double z2)
{
    return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1)+(z2-z1)*(z2-z1));
}
// 角度限制在-pi～pi之间
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
        if (c++ > 3) { //先做判断再++
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

// “虚拟”的位置，在实际的状态机运行中，并不使用Setpoint这个msg
// 获得四个期望点的位置，并由L和R点位置计算航向期望
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
            #ifdef NO_ROS_DEBUG
                ROS_INFO("setpoint index error!");
            #endif
            break;
    }

    // 计算出ENU下的航向期望
    /* calculate yaw*. -libn */
    deta_x = setpoint_R.pose.position.x - setpoint_L.pose.position.x;
    deta_y = setpoint_R.pose.position.y - setpoint_L.pose.position.y;
    yaw_sp_calculated_m2p_data.yaw_sp = atan2(deta_y,deta_x);
    yaw_sp_calculated_m2p_data.yaw_sp = wrap_pi(yaw_sp_calculated_m2p_data.yaw_sp + M_PI/2);    /* yaw* in ENU in rad within [-pi,pi]. */
    #ifdef NO_ROS_DEBUG
        ROS_INFO("yaw*(ENU) calculated for test with send4setpoint running.");
    #endif

    // 这里更新的要发布位置的航向期望，四元数旋转
    /* yaw* for controller. */
    pose_pub.pose.orientation.x = 0;			/* orientation expressed using quaternion. -libn */
    pose_pub.pose.orientation.y = 0;			/* w = cos(theta/2), x = nx * sin(theta/2),  y = ny * sin(theta/2), z = nz * sin(theta/2) -libn */
    pose_pub.pose.orientation.z = sin(yaw_sp_calculated_m2p_data.yaw_sp/2);
    pose_pub.pose.orientation.w = cos(yaw_sp_calculated_m2p_data.yaw_sp/2);		/* set yaw* = 90 degree(default in simulation). -libn */

    // 航向期望发布给Pix
    /* publish yaw_sp to pixhawk. */
    yaw_sp_calculated_m2p_pub.publish(yaw_sp_calculated_m2p_data);
    #ifdef NO_ROS_DEBUG
        ROS_INFO("publishing yaw_sp_calculated_m2p(ENU): %f",
                  yaw_sp_calculated_m2p_data.yaw_sp);
    #endif
}

// 切换视觉识别模式
/*  camera_switch: 0: mission closed;
 *                 1: vision_one_num_get;
 *                 2: vision_num_scan. -libn */
std_msgs::Int32 camera_switch_data;
ros::Publisher  camera_switch_pub;
// 下面两个订阅，消息由节点get_board_position发布
// 识别到的显示屏上的数字
std_msgs::Int32 vision_num_data;
void vision_num_cb(const std_msgs::Int32::ConstPtr& msg){
    vision_num_data = *msg;
    current_mission_num = vision_num_data.data;
    #ifdef NO_ROS_DEBUG
        ROS_INFO("subscribing vision_num_data = %d", vision_num_data.data);
    #endif
}
// 识别出的喷绘板数字以及位置
/* 10 drawing board positions. -libn */
state_machine::DrawingBoard10 board10;
void board_pos_cb(const state_machine::DrawingBoard10::ConstPtr& msg)
{
    // 如果杆伸了出来，喷绘板位置不更新
    /* stop update while in operation. */
    if(current_mission_state != mission_arm_spread &&
            current_mission_state != mission_num_hover_spray)
    {
        board10 = *msg;
    }
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

// 订阅实时位置信息与速度信息，以及相应的回调函数
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

/* transform to ENU from NED. */ //NED->ENU
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

// 订阅固定位置点，计算期望航向，并将固定位置点返回
/* subscribe messages from pixhawk. -libn */
state_machine::FIXED_TARGET_POSITION_P2M fixed_target_position_p2m_data;
ros::Publisher  fixed_target_return_m2p_pub;
state_machine::FIXED_TARGET_RETURN_M2P fixed_target_return_m2p_data;
/* get 4 setpoints and calculate yaw*. */
void fixed_target_position_p2m_cb(const state_machine::FIXED_TARGET_POSITION_P2M::ConstPtr& msg){
	fixed_target_position_p2m_data = *msg;
    #ifdef NO_ROS_DEBUG
        ROS_INFO("subscribing fixed_target_position_p2m: %5.3f %5.3f %5.3f",
                  fixed_target_position_p2m_data.home_x,
                  fixed_target_position_p2m_data.home_y,
                  fixed_target_position_p2m_data.home_z);
    #endif

    // 将获得的“NED坐标系”下的位置返回
	/* publish messages to pixhawk. -libn */
    fixed_target_return_m2p_data.home_x = fixed_target_position_p2m_data.home_x;
    fixed_target_return_m2p_data.home_y = fixed_target_position_p2m_data.home_y;
    fixed_target_return_m2p_data.home_z = -TAKEOFF_HEIGHT;
    fixed_target_return_m2p_data.observe_x = fixed_target_position_p2m_data.observe_x;
    fixed_target_return_m2p_data.observe_y = fixed_target_position_p2m_data.observe_y;
    fixed_target_return_m2p_data.observe_z = -FIXED_POS_HEIGHT;
    fixed_target_return_m2p_data.spray_left_x = fixed_target_position_p2m_data.spray_left_x;
    fixed_target_return_m2p_data.spray_left_y = fixed_target_position_p2m_data.spray_left_y;
    fixed_target_return_m2p_data.spray_left_z = -SCAN_HEIGHT;
    fixed_target_return_m2p_data.spray_right_x = fixed_target_position_p2m_data.spray_right_x;
    fixed_target_return_m2p_data.spray_right_y = fixed_target_position_p2m_data.spray_right_y;
    fixed_target_return_m2p_data.spray_right_z = -SCAN_HEIGHT;
    fixed_target_return_m2p_pub.publish(fixed_target_return_m2p_data);
    #ifdef NO_ROS_DEBUG	
        ROS_INFO("publishing fixed_target_return_m2p(NED): %f\t%f\t%f\t",
                  fixed_target_return_m2p_data.home_x,
                  fixed_target_return_m2p_data.home_y,
                  fixed_target_return_m2p_data.home_z);
    #endif

    // NED下的位置期望需要转化到ENU下
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

    // 通过ENU下的位置期望，计算航向期望
    /* calculate yaw*. -libn */
    deta_x = setpoint_R.pose.position.x - setpoint_L.pose.position.x;
    deta_y = setpoint_R.pose.position.y - setpoint_L.pose.position.y;
    yaw_sp_calculated_m2p_data.yaw_sp = atan2(deta_y,deta_x);
    // ENU下的航向期望！！！
    yaw_sp_calculated_m2p_data.yaw_sp = wrap_pi(yaw_sp_calculated_m2p_data.yaw_sp + M_PI/2);    /* yaw* in NED in rad within [-pi,pi]. */
    #ifdef NO_ROS_DEBUG
        ROS_INFO("yaw*(ENU) calculated using fixed_position from GCS.");
    #endif
    /* yaw* for controller. */
    pose_pub.pose.orientation.x = 0;			/* orientation expressed using quaternion. -libn */
    pose_pub.pose.orientation.y = 0;			/* w = cos(theta/2), x = nx * sin(theta/2),  y = ny * sin(theta/2), z = nz * sin(theta/2) -libn */
    pose_pub.pose.orientation.z = sin(yaw_sp_calculated_m2p_data.yaw_sp/2);
    pose_pub.pose.orientation.w = cos(yaw_sp_calculated_m2p_data.yaw_sp/2);		/* set yaw* = 90 degree(default in simulation). -libn */

    /* publish yaw_sp to pixhawk. */
    // NED下的航向期望
    yaw_sp_pub2GCS.yaw_sp = wrap_pi(-(yaw_sp_calculated_m2p_data.yaw_sp - M_PI/2));
    yaw_sp_calculated_m2p_pub.publish(yaw_sp_pub2GCS);
    #ifdef NO_ROS_DEBUG
        ROS_INFO("publishing yaw_sp_calculated_m2p(ENU): %f",
                  yaw_sp_calculated_m2p_data.yaw_sp);
    #endif
}

// 订阅修改任务的msg
// 修改状态的变量没有用到，变量spray_duration会被返回，用于监视
state_machine::TASK_STATUS_MONITOR_M2P task_status_monitor_m2p_data;
state_machine::TASK_STATUS_CHANGE_P2M task_status_change_p2m_data;
void task_status_change_p2m_cb(const state_machine::TASK_STATUS_CHANGE_P2M::ConstPtr& msg){
	task_status_change_p2m_data = *msg;
    #ifdef NO_ROS_DEBUG
        ROS_INFO("subscribing task_status_change_p2m: %5.3f %d %d",
                  task_status_change_p2m_data.spray_duration,
                  task_status_change_p2m_data.task_status,
                  task_status_change_p2m_data.loop_value);
    #endif
    task_status_monitor_m2p_data.spray_duration = task_status_change_p2m_data.spray_duration;
}

//void vision_one_num_get_cal(void)
//{
//	vision_one_num_get_m2p_data.loop_value = loop;
////	vision_one_num_get_m2p_data.num = ;	//TODO: add vision_num_get topic!
//}
//void yaw_sp_cal(void)
//{
//	yaw_sp_calculated_m2p_data.yaw_sp = 0.6f;
//}

/* publish messages to pixhawk. -libn */
state_machine::VISION_ONE_NUM_GET_M2P vision_one_num_get_m2p_data; //发布显示屏数字
state_machine::VISION_NUM_SCAN_M2P vision_num_scan_m2p_data;       //发布喷绘板的位置和数字
state_machine::OBSTACLE_POSITION_M2P obstacle_position_m2p_data;   //障碍物位置（没用）

/******************** 主函数 ********************/
int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(10.0);

    // --- 订阅 --- //
    // 飞行状态
    ros::Subscriber state_sub = nh.subscribe<state_machine::State>("mavros/state", 10, state_cb);
    /* receive indexed setpoint. -libn */ //实际任务中，这个订阅没有用到
    ros::Subscriber setpoint_Indexed_sub = nh.subscribe("Setpoint_Indexed", 100 ,SetpointIndexedCallback);
    /* get pixhawk's local position. -libn */ //飞机位置
    ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, pos_cb);
    /* get pixhawk's local velocity. -libn */ //飞机速度
    ros::Subscriber local_vel_sub = nh.subscribe<geometry_msgs::TwistStamped>("mavros/local_position/velocity", 10, vel_cb);
    /* get vision_num */ //显示屏数字
    ros::Subscriber vision_num_sub = nh.subscribe<std_msgs::Int32>("vision_num", 10, vision_num_cb);
    // 10个喷绘板的数字和位置
    ros::Subscriber DrawingBoard_Position_sub = nh.subscribe<state_machine::DrawingBoard10>("DrawingBoard_Position10", 10, board_pos_cb);
    board10.drawingboard.resize(10);		/* MUST! -libn */
    /* subscribe messages from pixhawk. -libn */ //固定的位置期望点
    ros::Subscriber fixed_target_position_p2m_sub = nh.subscribe<state_machine::FIXED_TARGET_POSITION_P2M>("mavros/fixed_target_position_p2m", 10, fixed_target_position_p2m_cb);
    // 任务状态修改信息，主要是喷绘时长
    ros::Subscriber task_status_change_p2m_sub = nh.subscribe<state_machine::TASK_STATUS_CHANGE_P2M>("mavros/task_status_change_p2m", 10, task_status_change_p2m_cb);

    // --- 发布 --- //
    // 发布位置期望，起飞时发布速度期望（定时发送）
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    ros::Publisher local_vel_pub = nh.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 10);
    // 下面两个要发布的service在实际飞行中不会遇到
    ros::ServiceClient arming_client = nh.serviceClient<state_machine::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<state_machine::SetMode>("mavros/set_mode");
    // takeoff and land service //降落所要使用的service（条件发送）
    ros::ServiceClient land_client = nh.serviceClient<state_machine::CommandTOL>("mavros/cmd/land");
    state_machine::CommandTOL landing_cmd;
    landing_cmd.request.min_pitch = 1.0;
    // 向Pix返回收到的固定位置点（立刻返回）
    fixed_target_return_m2p_pub  = nh.advertise<state_machine::FIXED_TARGET_RETURN_M2P>("mavros/fixed_target_return_m2p", 10);
    // 返回计算得到的航向期望（立刻返回）
    yaw_sp_calculated_m2p_pub  = nh.advertise<state_machine::YAW_SP_CALCULATED_M2P>("mavros/yaw_sp_calculated_m2p", 10);
    // 障碍物信息（没有用到）
    ros::Publisher  obstacle_position_m2p_pub  = nh.advertise<state_machine::OBSTACLE_POSITION_M2P>("mavros/obstacle_position_m2p", 10);
    // 回发任务状态（定时返回）
    ros::Publisher  task_status_monitor_m2p_pub  = nh.advertise<state_machine::TASK_STATUS_MONITOR_M2P>("mavros/task_status_monitor_m2p", 10);
    // 回发喷绘板位置信息（定时返回）
    ros::Publisher  vision_num_scan_m2p_pub  = nh.advertise<state_machine::VISION_NUM_SCAN_M2P>("mavros/vision_num_scan_m2p", 10);
    // 回发显示屏信息（定时返回）
    ros::Publisher  vision_one_num_get_m2p_pub  = nh.advertise<state_machine::VISION_ONE_NUM_GET_M2P>("mavros/vision_one_num_get_m2p", 10);
    // 改变视觉算法工作模式（条件返回）
    camera_switch_pub  = nh.advertise<std_msgs::Int32>("camera_switch", 10);
    camera_switch_data.data = 0;

    // 如果与Pix没连接上，将一直在这个循环里
    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    // 正式启动之前，发送期望位置点，保证Pix能解锁
    if(1)   /* initialisation: send 100 setpoints. */
    {
        /* local velocity setpoint publish. -libn */
        vel_pub.twist.linear.x = 0.0f;
        vel_pub.twist.linear.y = 0.0f;
        vel_pub.twist.linear.z = 2.0f;
        vel_pub.twist.angular.x = 0.0f; //航向保持不变
        vel_pub.twist.angular.y = 0.0f;
        vel_pub.twist.angular.z = 0.0f;
        ROS_INFO("sending 100 setpoints, please wait 10 seconds!");
        //send a few setpoints before starting
        for(int i = 100; ros::ok() && i > 0; --i){
            local_vel_pub.publish(vel_pub); //因为起飞是使用速度环控制，所以这里也发布速度期望点
            ros::spinOnce();
            rate.sleep();
        }
    }
    ROS_INFO("Initialization finished!");

    ros::Time last_request = ros::Time::now(); //任务执行要使用的时间戳（都没什么作用）
    ros::Time last_show_request = ros::Time::now();
    last_state_display = current_state; //任务执行时，要显示的状态
    last_state.mode = current_state.mode;
    last_state.armed = current_state.armed;
    #ifdef NO_ROS_DEBUG
        ROS_INFO("current_state.mode = %s",current_state.mode.c_str());
        ROS_INFO("armed status: %d",current_state.armed);
    #endif

    // 各种变量的初始化
    /* initialisation(loop once). */
    /* initialize: 4 fixed setpoints(A,L,R,D), yaw*, 10 board position,
     * current_mission_num, camera_switch_data. */
    if(1)
    {
        setpoint_H.pose.position.x = current_pos.pose.position.x;
        setpoint_H.pose.position.y = current_pos.pose.position.y;
        setpoint_H.pose.position.z = TAKEOFF_HEIGHT;  /* it's better to choose z* = SAFE_HEIGHT_DISTANCE(no altitude lost). */
        setpoint_A.pose.position.x = 0.0f;
        setpoint_A.pose.position.y = 0.0f;
        setpoint_A.pose.position.z = FIXED_POS_HEIGHT;
        setpoint_L.pose.position.x = 0.0f;
        setpoint_L.pose.position.y = 0.0f;
        setpoint_L.pose.position.z = SCAN_HEIGHT;
        setpoint_R.pose.position.x = 0.0f;
        setpoint_R.pose.position.y = 0.0f;
        setpoint_R.pose.position.z = SCAN_HEIGHT;
        setpoint_D.pose.position.x = 0.0f;
        setpoint_D.pose.position.y = 0.0f;
        setpoint_D.pose.position.z = FIXED_POS_HEIGHT;

        yaw_sp_calculated_m2p_data.yaw_sp = 90*M_PI/180;   /* default yaw*(90 degree)(ENU) -> North! */
        /* publish yaw_sp to pixhawk. */
        yaw_sp_calculated_m2p_pub.publish(yaw_sp_calculated_m2p_data);
        #ifdef NO_ROS_DEBUG
            ROS_INFO("publishing yaw_sp_calculated_m2p: %f",
                      yaw_sp_calculated_m2p_data.yaw_sp);
        #endif
        /* yaw* for controller. */ //yaw_sp_calculated_m2p_data.yaw_sp中存的实际上是ENU下的航向期望
        pose_pub.pose.orientation.x = 0;			/* orientation expressed using quaternion. -libn */
        pose_pub.pose.orientation.y = 0;			/* w = cos(theta/2), x = nx * sin(theta/2),  y = ny * sin(theta/2), z = nz * sin(theta/2) -libn */
        pose_pub.pose.orientation.z = sin(yaw_sp_calculated_m2p_data.yaw_sp/2);
        pose_pub.pose.orientation.w = cos(yaw_sp_calculated_m2p_data.yaw_sp/2);

        // 默认所有喷绘板的数字都是不可见的，在回调函数中，会对变量进行更新
        for(int co = 0; co<10; ++co)
        {
            /* set param valid true as default to make state machine able to run, and I am sure all numbers will be valid!(vision) */
            board10.drawingboard[co].valid = false;  /* Normly it should be false as the default setting. */
            board10.drawingboard[co].x = 0.0f;
            board10.drawingboard[co].y = 0.0f;
            board10.drawingboard[co].z = 0.0f;  /* it's safe for we have SAFE_HEIGHT_DISTANCE. */
        }
        // 默认的显示屏数字
        current_mission_num = -1;    /* set current_mission_num as 0 as default. */
        last_mission_num = -1;

        /* camera_switch: 0: mission closed;
         *                1: vision_one_num_get;
         *                2: vision_num_scan. -libn */
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
            // 手动模式下识别显示屏
            if(current_state.mode == "MANUAL" && last_state.mode != "MANUAL")
            {
                last_state.mode = "MANUAL"; //上一状态值赋值为当前状态值，实现单次msg发送或消息显示
                #ifdef NO_ROS_DEBUG
                    ROS_INFO("switch to mode: MANUAL");
                #endif
                /* start manual scanning. -libn */
                /*  camera_switch/camera_switch_return: 0: mission closed; 1: vision_one_num_get; 2: vision_num_scan. -libn */
                camera_switch_data.data = 1;
                camera_switch_pub.publish(camera_switch_data);
                #ifdef NO_ROS_DEBUG
                    ROS_INFO("send camera_switch_data = %d",(int)camera_switch_data.data);
                #endif
            }
            // 定高模式下识别喷绘板
            if(current_state.mode == "ALTCTL" && last_state.mode != "ALTCTL")
            {
                last_state.mode = "ALTCTL";
                #ifdef NO_ROS_DEBUG
                    ROS_INFO("switch to mode: ALTCTL");
                #endif
                /* start manual scanning. -libn */
                /*  camera_switch/camera_switch_return: 0: mission closed; 1: vision_one_num_get; 2: vision_num_scan. -libn */
                camera_switch_data.data = 2;
                camera_switch_pub.publish(camera_switch_data);
                #ifdef NO_ROS_DEBUG
                    ROS_INFO("send camera_switch_data = %d",(int)camera_switch_data.data);
                #endif

            }
            // 显示进入offboard模式
            if(current_state.mode == "OFFBOARD" && last_state.mode != "OFFBOARD")
            {
                last_state.mode = "OFFBOARD";
                #ifdef NO_ROS_DEBUG
                    ROS_INFO("switch to mode: OFFBOARD");
                #endif

            }
            // 显示飞行器被armed
            if(current_state.armed && !last_state.armed)
            {
                last_state.armed = current_state.armed;
                #ifdef NO_ROS_DEBUG
                ROS_INFO("UAV armed!");
                #endif
            }
        }
        // 进入手动模式并且armed，将相机的工作关掉，并且当前任务值设置为一个负数
        if(current_state.mode == "MANUAL" && current_state.armed)
        {
            camera_switch_data.data = 0;    /* disable camere. */
            camera_switch_pub.publish(camera_switch_data);
            #ifdef NO_ROS_DEBUG
                ROS_INFO("send camera_switch_data = %d",(int)camera_switch_data.data);
            #endif
            current_mission_num = -1;    /* set current_mission_num as -1 as default. */
            last_mission_num = -1;   /* disable the initial mission_num gotten before takeoff. */
        }

        // 发布自动降落service
        // landing
		if(current_state.armed && current_mission_state == land)	/* set landing mode until uav stops. -libn */
		{
            if(current_state.mode != "MANUAL" && current_state.mode != "AUTO.LAND" &&
               (ros::Time::now() - last_request > ros::Duration(5.0)))
			{
				if(land_client.call(landing_cmd) && landing_cmd.response.success)
				{
					ROS_INFO("AUTO LANDING!");
				}
				last_request = ros::Time::now();
			}
		}

        // 进入offboard模式并且armed，将开始运行状态机，并显示任务状态
        /* state_machine start and mission state display. -libn */
		if(current_state.mode == "OFFBOARD" && current_state.armed)	/* set message display delay(0.5s). -libn */
		{
            #ifdef NO_ROS_DEBUG
                ROS_INFO("now I am in OFFBOARD and armed mode!");	/* state machine! -libn */
            #endif

            /***** 状态机函数 *****/
			state_machine_func();

            // 任务的时序控制，仿制子任务和总任务超时
            /* system timer. TODO! */
            if(1)
            {
                // 总任务超时，就会强制回到home点，并进行降落
                /* set time deadline. */
                /* mission timer(5 loops). -libn */
                if(ros::Time::now() > mission_timer_start_time + ros::Duration(MAX_FLIGHT_TIME) &&
                   force_home_enable == true)
                {
                    force_home_enable = false; /* force once! */
                    current_mission_state = mission_force_return_home;	/* mission timeout. -libn */
                    #ifdef NO_ROS_DEBUG
                        ROS_INFO("mission time out! -> return to home!");
                    #endif
                    mission_last_time = ros::Time::now();   /* start counting time(for hovering). */
                }

                // 如果子任务超时了，但是总任务没有超时，那就需要继续运行下一个子任务
                // 记录当前子任务超时的任务状态和子任务数字，任务计数+1，要回到观察点
                /* subtask timer(1 loop). -libn */
                if(subtimer_start)
                {
                    if(ros::Time::now() > current_subtask_time + ros::Duration(SUB_PERIOD) &&
                       ros::Time::now() < mission_timer_start_time + ros::Duration(MAX_FLIGHT_TIME) &&
                       loop <= 5)  /* stop subtask timer when dealing with failures. */
                    {
                        subtimer_start = false;
                        // 判断一些可强制回到观察点的情况
                        if(loop == 0)
                        {
                            #ifdef NO_ROS_DEBUG
                            ROS_INFO("loop timeout -> start spray task");
                            #endif
                            current_mission_state = mission_observe_point_go;
                        }
                        if((current_mission_state >= mission_num_search && current_mission_state <= mission_arm_spread) ||
                           current_mission_state == mission_hover_before_spary)
                        {
                            /* error recorded! */
                            mission_failure_acount++;
                            failure[mission_failure_acount-1].num = current_mission_num;
                            failure[mission_failure_acount-1].state = current_mission_state;
                            /* TODO: mission failure recorded(using switch/case). -libn */
                            #ifdef NO_ROS_DEBUG
                                ROS_INFO("loop timeout -> start next loop");
                            #endif
                            current_mission_state = mission_observe_point_go;	/* loop timeout, forced to switch to next loop. -libn */
                            current_mission_num = -1; //新的子任务，新的显示屏数字识别
                        }
                    }
                }

                // 子任务定时器会在mission_observe_point_go中关掉，
                // 在mission_observe_num_wait中识别到新的数字后重新开启，
                // 如果在这期间没能识别到，需要在数字显示完成后，进入下一次的子任务
                if((current_mission_state == mission_observe_point_go || current_mission_state == mission_observe_num_wait) &&
                   loop <= 5 && ros::Time::now() > current_subtask_time + ros::Duration(SUB_PERIOD + 20))
                {
                    if(loop >= 1 && loop <= 4)
                    {
                        current_subtask_time = current_subtask_time + ros::Duration(60);
                        current_mission_state = mission_observe_point_go;
                    }
                    if(loop ==5)
                    {
                        loop++;
                        current_mission_state=mission_num_done;
                    }
                }
            }

            // ROS消息显示
            if(1)   /* ROS_INFO display. */
            {
                // 当前任务状态
                #ifdef NO_ROS_DEBUG
                    ROS_INFO("current loop: %d",loop);
                    ROS_INFO("current_mission_state: %d",current_mission_state);
                #endif
                // 速度或位置期望
                if(velocity_control_enable)
                {
                    #ifdef NO_ROS_DEBUG
                        ROS_INFO("velocity*: %5.3f %5.3f %5.3f",vel_pub.twist.linear.x, vel_pub.twist.linear.y, vel_pub.twist.linear.z);
                    #endif
                }
                else
                {
                    #ifdef NO_ROS_DEBUG
                        ROS_INFO("position*: %5.3f %5.3f %5.3f",pose_pub.pose.position.x,pose_pub.pose.position.y,pose_pub.pose.position.z);
                    #endif
                }
                // 当前所要执行的任务
                #ifdef NO_ROS_DEBUG
                    ROS_INFO("current position: %5.3f %5.3f %5.3f\n",current_pos.pose.position.x,current_pos.pose.position.y,current_pos.pose.position.z);
                    ROS_INFO("current_mission_num = %d",current_mission_num);
                    ROS_INFO("board: current_mission_num: %d\n"
                             "position:%5.3f %5.3f %5.3f",current_mission_num,board10.drawingboard[current_mission_num].x,
                             board10.drawingboard[current_mission_num].y,board10.drawingboard[current_mission_num].z);
                    ROS_INFO("spray time = %f",(float)task_status_change_p2m_data.spray_duration);
                #endif
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
        // 其他情况下，如果模式改变或者armed状态改变，也会显示一些消息
        else    /* All information display. */
		{
            if(current_state.mode != last_state_display.mode || last_state_display.armed != current_state.armed)
			{
                last_state_display.armed = current_state.armed;
                last_state_display.mode = current_state.mode;

                #ifdef NO_ROS_DEBUG
                    ROS_INFO("current_state.mode = %s",current_state.mode.c_str());
                    ROS_INFO("last_state_display.mode = %s",last_state_display.mode.c_str());
                    ROS_INFO("armed status: %d\n",current_state.armed);
                #endif
                #ifdef NO_ROS_DEBUG
                    ROS_INFO("current position: %5.3f %5.3f %5.3f", current_pos.pose.position.x,
                                                                    current_pos.pose.position.y,
                                                                    current_pos.pose.position.z);
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
                    ROS_INFO("yaw_sp(ENU) = rad:%f deg:%f",yaw_sp_calculated_m2p_data.yaw_sp,
                                                           yaw_sp_calculated_m2p_data.yaw_sp*180/M_PI);
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
 //                 ROS_INFO("SCREEN_HEIGHT = %d SAFE_HEIGHT_DISTANCE = %d",(int)SCREEN_HEIGHT,(int)SAFE_HEIGHT_DISTANCE);
                    ROS_INFO("SAFE_HEIGHT_DISTANCE = %d",(int)SAFE_HEIGHT_DISTANCE);
                #endif
			}
		}

        // 位置控制下，如果期望位置相距实际位置过远...
        // 这里的处理没有用到
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

        // 向pix回发大量的数据，用于地面站的监督
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
            // 返回任务状态信息
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
            // 每次返回一个喷绘板的位置信息
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
            // 返回当前显示屏数字
            vision_one_num_get_m2p_data.loop_value = loop;
            vision_one_num_get_m2p_data.num = current_mission_num; //这样会使得下一个loop在获得新的显示屏数字前，
                                                                   //保持上一个loop的数字
            vision_one_num_get_m2p_pub.publish(vision_one_num_get_m2p_data);
    //		ROS_INFO("publishing vision_one_num_get_m2p: %d %d",
    //				vision_one_num_get_m2p_data.loop_value,
    //				vision_one_num_get_m2p_data.num);
        }

        // 发布期望速度或位置，期望值在任务状态机中获取
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
            /* start mission_timer once. */ //第一次进入起飞模式，表示任务开始，定时器计时
            if(mission_timer_enable)
            {
                mission_timer_start_time = ros::Time::now();    /* start mission_timer. */
                current_subtask_time = mission_timer_start_time - ros::Duration(10);
                mission_timer_enable = false;
            }
			/* local velocity setpoint publish. -libn */
            velocity_control_enable = true; //速度环控制
            vel_pub.twist.linear.x = 0.0f;
			vel_pub.twist.linear.y = 0.0f;
            vel_pub.twist.linear.z = 2.2f; //速度环起飞
			vel_pub.twist.angular.x = 0.0f;
			vel_pub.twist.angular.y = 0.0f;
			vel_pub.twist.angular.z = 0.0f;
            if(current_vel.twist.linear.z > 0.5 && current_pos.pose.position.z > 0.8)
            {
                #ifdef NO_ROS_DEBUG
                    ROS_INFO("current_vel.twist.linear.x = %f",current_vel.twist.linear.x);
                #endif
                // 起飞后旋停
                current_mission_state = mission_hover_after_takeoff;
                mission_last_time = ros::Time::now();
                velocity_control_enable = false;
                pose_pub.pose.position.x = current_pos.pose.position.x;
                pose_pub.pose.position.y = current_pos.pose.position.y;
                pose_pub.pose.position.z = setpoint_H.pose.position.z; //1.8m
                /*  camera_switch: 0: mission closed; 1: vision_one_num_get; 2: vision_num_scan. -libn */
                camera_switch_data.data = 0;
                camera_switch_pub.publish(camera_switch_data);
                #ifdef NO_ROS_DEBUG
                ROS_INFO("send camera_switch_data = %d",(int)camera_switch_data.data);
                #endif
            }
        break;
        case mission_hover_after_takeoff:
            pose_pub.pose.position.x = current_pos.pose.position.x; //没有位置控制
        	pose_pub.pose.position.y = current_pos.pose.position.y;
        	pose_pub.pose.position.z = setpoint_H.pose.position.z;
            if(ros::Time::now() - mission_last_time > ros::Duration(3))	/* hover for 5 seconds. -libn */
        	{
                current_mission_state = mission_scan_left_go;
        	}
        break;
        // 预扫阶段，获得喷绘板的位置
        /* add scan mission  --start. */
        case mission_scan_left_go:
            // 保持一定的扫描距离SCAN_VISION_DISTANCE
            pose_pub.pose.position.x = setpoint_L.pose.position.x - SCAN_VISION_DISTANCE * cos(yaw_sp_calculated_m2p_data.yaw_sp);
            pose_pub.pose.position.y = setpoint_L.pose.position.y - SCAN_VISION_DISTANCE * sin(yaw_sp_calculated_m2p_data.yaw_sp);
            pose_pub.pose.position.z = SCAN_HEIGHT;
//            if((abs(current_pos.pose.position.x - pose_pub.pose.position.x) < 0.2) &&      // switch to next state
//               (abs(current_pos.pose.position.y - pose_pub.pose.position.y) < 0.2) &&
//               (abs(current_pos.pose.position.z - pose_pub.pose.position.z) < 0.2))
            // 到达喷绘点左点后，开始向右点移动
            if(circle_distance(current_pos.pose.position.x,pose_pub.pose.position.x,
                               current_pos.pose.position.y,pose_pub.pose.position.y,
                               current_pos.pose.position.z,pose_pub.pose.position.z) < 0.2)
            {
                current_mission_state = mission_scan_right_move;
                mission_last_time = ros::Time::now();
                /*  camera_switch: 0: mission closed; 1: vision_one_num_get; 2: vision_num_scan. -libn */
                camera_switch_data.data = 2;
                camera_switch_pub.publish(camera_switch_data);
                #ifdef NO_ROS_DEBUG
                    ROS_INFO("send camera_switch_data = %d",(int)camera_switch_data.data);
                #endif
             }
        break;
        case mission_scan_right_move:
            // 向右移动，目标值设置为右喷绘点
            // 移动过程中，已在pix中限制了一个较小的速度环期望，保证平滑扫描
            pose_pub.pose.position.x = setpoint_R.pose.position.x - SCAN_VISION_DISTANCE * cos(yaw_sp_calculated_m2p_data.yaw_sp);
            pose_pub.pose.position.y = setpoint_R.pose.position.y - SCAN_VISION_DISTANCE * sin(yaw_sp_calculated_m2p_data.yaw_sp);
            pose_pub.pose.position.z = SCAN_HEIGHT;
//            /* limit error(x,y) between current position and destination. */
//            if(abs(pose_pub.pose.position.x - current_pos.pose.position.x) > SCAN_MOVE_SPEED ||
//                abs(pose_pub.pose.position.y - current_pos.pose.position.y) > SCAN_MOVE_SPEED)
//            {
//                double error_temp[2] = {0,0};
//                error_limit(current_pos.pose.position.x,current_pos.pose.position.y,pose_pub.pose.position.x,pose_pub.pose.position.y,error_temp);
//                pose_pub.pose.position.x = current_pos.pose.position.x + SCAN_MOVE_SPEED*error_temp[0];
//                pose_pub.pose.position.y = current_pos.pose.position.y + SCAN_MOVE_SPEED*error_temp[1];
//            }

//            if((abs(current_pos.pose.position.x - pose_pub.pose.position.x) < 0.2) &&      // switch to next state
//               (abs(current_pos.pose.position.y - pose_pub.pose.position.y) < 0.2) &&
//               (abs(current_pos.pose.position.z - pose_pub.pose.position.z) < 0.2))
            if(circle_distance(current_pos.pose.position.x,pose_pub.pose.position.x,
                               current_pos.pose.position.y,pose_pub.pose.position.y,
                               current_pos.pose.position.z,pose_pub.pose.position.z) < 0.2)
            {
                current_mission_state = mission_scan_right_hover;
                mission_last_time = ros::Time::now();
            }
        break;
        case mission_scan_right_hover:
            pose_pub.pose.position.x = setpoint_R.pose.position.x - SCAN_VISION_DISTANCE * cos(yaw_sp_calculated_m2p_data.yaw_sp);
            pose_pub.pose.position.y = setpoint_R.pose.position.y - SCAN_VISION_DISTANCE * sin(yaw_sp_calculated_m2p_data.yaw_sp);
            pose_pub.pose.position.z = SCAN_HEIGHT;
            // 到达右点之后，会短暂旋停，之后会向左点移动
            if(ros::Time::now() - mission_last_time > ros::Duration(1))	/* hover for 5 seconds. -libn */
            {
                current_mission_state = mission_scan_left_move;
                mission_last_time = ros::Time::now();
                // 如果时喷涂阶段扫描到了需要的数字
                if(scan_to_get_pos && board10.drawingboard[current_mission_num].valid)
                {
                    current_mission_state = mission_num_locate;
                    scan_to_get_pos = false;
                }
            }
        break;
        case mission_scan_left_move:
            pose_pub.pose.position.x = setpoint_L.pose.position.x - SCAN_VISION_DISTANCE * cos(yaw_sp_calculated_m2p_data.yaw_sp);
            pose_pub.pose.position.y = setpoint_L.pose.position.y - SCAN_VISION_DISTANCE * sin(yaw_sp_calculated_m2p_data.yaw_sp);
            pose_pub.pose.position.z = SCAN_HEIGHT;
//            /* limit error(x,y) between current position and destination within [-1,1]. */
//            if(abs(pose_pub.pose.position.x - current_pos.pose.position.x) > SCAN_MOVE_SPEED ||
//                abs(pose_pub.pose.position.y - current_pos.pose.position.y) > SCAN_MOVE_SPEED)
//            {
//                double error_temp[2] = {0,0};
//                error_limit(current_pos.pose.position.x,current_pos.pose.position.y,pose_pub.pose.position.x,pose_pub.pose.position.y,error_temp);
//                pose_pub.pose.position.x = current_pos.pose.position.x + SCAN_MOVE_SPEED*error_temp[0];
//                pose_pub.pose.position.y = current_pos.pose.position.y + SCAN_MOVE_SPEED*error_temp[1];
//            }

//            if((abs(current_pos.pose.position.x - pose_pub.pose.position.x) < 0.2) &&      // switch to next state
//               (abs(current_pos.pose.position.y - pose_pub.pose.position.y) < 0.2) &&
//               (abs(current_pos.pose.position.z - pose_pub.pose.position.z) < 0.2))
            if(circle_distance(current_pos.pose.position.x,pose_pub.pose.position.x,
                               current_pos.pose.position.y,pose_pub.pose.position.y,
                               current_pos.pose.position.z,pose_pub.pose.position.z) < 0.2)
            {
                current_mission_state = mission_scan_left_hover;
                mission_last_time = ros::Time::now();
            }
        break;
        case mission_scan_left_hover:
            pose_pub.pose.position.x = setpoint_L.pose.position.x - SCAN_VISION_DISTANCE * cos(yaw_sp_calculated_m2p_data.yaw_sp);
            pose_pub.pose.position.y = setpoint_L.pose.position.y - SCAN_VISION_DISTANCE * sin(yaw_sp_calculated_m2p_data.yaw_sp);
            pose_pub.pose.position.z = SCAN_HEIGHT;
            if(ros::Time::now() - mission_last_time > ros::Duration(1))	/* hover for 5 seconds. -libn */
            {
                if(scan_to_get_pos) //如果是喷涂阶段的扫描
                {
                    if(board10.drawingboard[current_mission_num].valid) //获得需要的数字位置之后，可以继续向下执行
                    {
                        current_mission_state = mission_num_search;
                        scan_to_get_pos = false;
                    }
                    else //如果没有获得需要的数字位置，就继续扫描
                    {
                        current_mission_state = mission_scan_left_go;
                    }
                }
                else //到达左点之后，会短暂旋停，之后飞向观察点
                {
                    current_mission_state = mission_observe_point_go;
                    mission_last_time = ros::Time::now();
                }
                /*  camera_switch: 0: mission closed; 1: vision_one_num_get; 2: vision_num_scan. -libn */
                camera_switch_data.data = 0;
                camera_switch_pub.publish(camera_switch_data);
                #ifdef NO_ROS_DEBUG
                    ROS_INFO("send camera_switch_data = %d",(int)camera_switch_data.data);
                #endif
            }
        break;
        /* add scan mission  --stop. */
        // 此任务状态，代表了一个新的子任务的开始
        case mission_observe_point_go:
            loop++; //进入一次新的子任务循环
        	if(loop > 5)
			{
                current_mission_state = mission_num_done; //表示所有任务完成
				break;
			}
            subtimer_start = false; //开始回观察点，就不需要子任务定时器了
        	pose_pub.pose.position.x = setpoint_A.pose.position.x;
			pose_pub.pose.position.y = setpoint_A.pose.position.y;
            pose_pub.pose.position.z = setpoint_A.pose.position.z;
            /* camera_switch revised. */
            if((abs(current_pos.pose.position.x - pose_pub.pose.position.x) < 1.0) &&      // switch to next state
               (abs(current_pos.pose.position.y - pose_pub.pose.position.y) < 1.0))
            {
                /* camera_switch: 0: mission closed; 1: vision_one_num_get; 2: vision_num_scan. -libn */
                camera_switch_data.data = 1; //打开摄像机
                camera_switch_pub.publish(camera_switch_data);
                #ifdef NO_ROS_DEBUG
                    ROS_INFO("send camera_switch_data = %d",(int)camera_switch_data.data);
                #endif
            }
//            if((abs(current_pos.pose.position.x - pose_pub.pose.position.x) < 0.2) &&      // switch to next state
//               (abs(current_pos.pose.position.y - pose_pub.pose.position.y) < 0.2) &&
//               (abs(current_pos.pose.position.z - pose_pub.pose.position.z) < 0.2))
            if(circle_distance(current_pos.pose.position.x,pose_pub.pose.position.x,
                               current_pos.pose.position.y,pose_pub.pose.position.y,
                               current_pos.pose.position.z,pose_pub.pose.position.z) < 0.2)
            {
                current_mission_state = mission_observe_num_wait; //等待数字的出现
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
            // 没检测到新的数字，会一直进行旋停
            if(current_mission_num == -1 || current_mission_num == last_mission_num)
            {
                current_mission_state = mission_observe_num_wait; // stop mission_state++, avoid repeating spraying. ;
            }
            else    /* valid number detected. */
            {
                // 检测到新的数字，短时间的停留
                if(ros::Time::now() - mission_last_time > ros::Duration(1))	/* hover for 1 seconds. -libn */
                {
                    current_subtask_time = ros::Time::now(); //记录当前子任务的开始时间
                    subtimer_start = true; //子任务定时器开启
                    current_mission_state = mission_num_search;
                    /* change and publish camera_switch_data for next subtask. */
                    /*  camera_switch: 0: mission closed; 1: vision_one_num_get; 2: vision_num_scan. -libn */
                    camera_switch_data.data = 2;
                    camera_switch_pub.publish(camera_switch_data);
                    #ifdef NO_ROS_DEBUG
                        ROS_INFO("send camera_switch_data = %d",(int)camera_switch_data.data);
                    #endif
                    last_mission_num = current_mission_num;
                }
            }
        break;
        case mission_num_search:
//        	ROS_INFO("board10.drawingboard[current_mission_num].valid = %d",board10.drawingboard[current_mission_num].valid);
            // 如果所要执行的数字位置已知，直接飞到相应的数字前面
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
                pose_pub.pose.position.z = board10.drawingboard[current_mission_num].z + SAFE_HEIGHT_DISTANCE; //保持与数字的一定距离，并进行高度补偿
//				ROS_INFO("distance: x = %5.3f y = %5.3f z = %5.3f\n",
//						abs(current_pos.pose.position.x - board10.drawingboard[current_mission_num].x),
//						abs(current_pos.pose.position.y - board10.drawingboard[current_mission_num].y),
//						abs(current_pos.pose.position.z - (board10.drawingboard[current_mission_num].z+3)));

//                if((abs(current_pos.pose.position.x - pose_pub.pose.position.x) < 1) &&      // switch to next state
//                   (abs(current_pos.pose.position.y - pose_pub.pose.position.y) < 1) &&
//                   (abs(current_pos.pose.position.z - pose_pub.pose.position.z) < 1))
                if(circle_distance(current_pos.pose.position.x,pose_pub.pose.position.x,
                                   current_pos.pose.position.y,pose_pub.pose.position.y,
                                   current_pos.pose.position.z,pose_pub.pose.position.z) < 0.6)
                {
                    current_mission_state = mission_num_get_close; //到达数字前面之后，后靠近数字以进行喷涂
					mission_last_time = ros::Time::now();
                    #ifdef NO_ROS_DEBUG
                        ROS_INFO("mission switched well!");
                    #endif
				}
        	}
            else //如果不知道喷绘板的位置，需要再次预扫，以获得该位置
        	{
        		/* TODO: add scanning method! -libn */
                current_mission_state = mission_scan_left_go;
                scan_to_get_pos = true;
                #ifdef NO_ROS_DEBUG
                    ROS_INFO("fall into mission state: mission_num_scan_again");
                #endif
        	}
        break;
        case mission_num_locate: //这个任务状态相当于上一任务状态的扩展，到达后旋停（作用不大）
            pose_pub.pose.position.x = board10.drawingboard[current_mission_num].x - VISION_SCAN_DISTANCE * cos(yaw_sp_calculated_m2p_data.yaw_sp);	/* TODO:switch to different board positions. -libn */
            pose_pub.pose.position.y = board10.drawingboard[current_mission_num].y - VISION_SCAN_DISTANCE * sin(yaw_sp_calculated_m2p_data.yaw_sp);
            pose_pub.pose.position.z = board10.drawingboard[current_mission_num].z + SAFE_HEIGHT_DISTANCE;
            /* TODO:update the position of the drawing board.  -libn */
			relocate_valid = true;
			if(relocate_valid)
			{
                if(ros::Time::now() - mission_last_time > ros::Duration(1))	/* hover for 5 seconds. -libn */
                {
                    current_mission_state = mission_num_get_close;
                    mission_last_time = ros::Time::now();
                }
			}
//			else
//			{
//                if(ros::Time::now() - mission_last_time > ros::Duration(5))	/* wait for this operate 5 seconds at most. -libn */
//				{
//					/* TODO: add scanning method! -libn */
//                    current_mission_state = mission_num_scan_again; // current_mission_state++;
//				}
//			}
        break;
        case mission_num_get_close:
            // 靠近喷绘板，以提高定位精度
            pose_pub.pose.position.x = board10.drawingboard[current_mission_num].x - SPRAY_DISTANCE * cos(yaw_sp_calculated_m2p_data.yaw_sp);	/* TODO:switch to different board positions. -libn */
            pose_pub.pose.position.y = board10.drawingboard[current_mission_num].y - SPRAY_DISTANCE * sin(yaw_sp_calculated_m2p_data.yaw_sp);
            pose_pub.pose.position.z = board10.drawingboard[current_mission_num].z + SAFE_HEIGHT_DISTANCE;
//            if((abs(current_pos.pose.position.x - pose_pub.pose.position.x) < 0.2) &&      // switch to next state
//               (abs(current_pos.pose.position.y - pose_pub.pose.position.y) < 0.2) &&
//               (abs(current_pos.pose.position.z - pose_pub.pose.position.z) < 0.2))
            if(circle_distance(current_pos.pose.position.x,pose_pub.pose.position.x,
                               current_pos.pose.position.y,pose_pub.pose.position.y,
                               current_pos.pose.position.z,pose_pub.pose.position.z) < 0.15)
            {
                current_mission_state = mission_hover_before_spary;
                mission_last_time = ros::Time::now(); //更新时间戳
            }
        break;
        case mission_hover_before_spary: //喷扫前的旋停，更新喷绘板的高精度位置
            static int hover_count = 0;
            static int hover_acc_count = 0;
            pose_pub.pose.position.x = board10.drawingboard[current_mission_num].x - SPRAY_DISTANCE * cos(yaw_sp_calculated_m2p_data.yaw_sp);	/* TODO:switch to different board positions. -libn */
            pose_pub.pose.position.y = board10.drawingboard[current_mission_num].y - SPRAY_DISTANCE * sin(yaw_sp_calculated_m2p_data.yaw_sp);
            pose_pub.pose.position.z = board10.drawingboard[current_mission_num].z + SAFE_HEIGHT_DISTANCE;
            // 先旋停2s
            if((ros::Time::now() - mission_last_time > ros::Duration(2)) && (hover_count == 0))
            {
                hover_count++;
            }
            // 2s后才能进入下列语句，每0.5s检测与目标点的距离
            // 当距离足够短的连续次数大于3次，就可以进行喷洒了
            // 否则，统计检测的次数，如果检测次数过多，表明由外界干扰，直接进行强喷
            if((ros::Time::now() - mission_last_time > ros::Duration(0.5)) && (hover_count > 0))
            {
//                if((abs(current_pos.pose.position.x - pose_pub.pose.position.x) < 0.1) &&
//                   (abs(current_pos.pose.position.y - pose_pub.pose.position.y) < 0.1) &&
//                   (abs(current_pos.pose.position.z - pose_pub.pose.position.z) < 0.1))
                if(circle_distance(current_pos.pose.position.x,pose_pub.pose.position.x,
                                   current_pos.pose.position.y,pose_pub.pose.position.y,
                                   current_pos.pose.position.z,pose_pub.pose.position.z) < 0.1)
                {
                    hover_acc_count++;
                    if(hover_acc_count > 3)
                    {
                        current_mission_state = mission_arm_spread;
                        hover_count = 0;
                        hover_acc_count = 0;
                        mission_last_time = ros::Time::now();
                        break;
                    }
                }
                else
                {
                    hover_acc_count = 0;
                }
                /* count for max time */
                hover_count++;
                if(hover_count > 10)
                {
                    /* force spray */
                    current_mission_state = mission_arm_spread;
                    hover_count = 0;
                    hover_acc_count = 0;
                    mission_last_time = ros::Time::now();
                    break;
                }
                mission_last_time = ros::Time::now();
                /* TODO: start spraying. -libn */
            }
        break;
        case mission_arm_spread: //此状态下，喷绘板位置不再进行更新
            static int loop_count = 0;
            static int acc_count = 0;
            pose_pub.pose.position.x = board10.drawingboard[current_mission_num].x - SPRAY_DISTANCE * cos(yaw_sp_calculated_m2p_data.yaw_sp);	/* TODO:switch to different board positions. -libn */
            pose_pub.pose.position.y = board10.drawingboard[current_mission_num].y - SPRAY_DISTANCE * sin(yaw_sp_calculated_m2p_data.yaw_sp);
            pose_pub.pose.position.z = board10.drawingboard[current_mission_num].z + SAFE_HEIGHT_DISTANCE;
            // 先旋停1.5s
            if((ros::Time::now() - mission_last_time > ros::Duration(1.5)) && (loop_count == 0))
            {
                loop_count++;
            }
            // 1.5s后才进行下列语句，同样设置了强喷策略，避免受到外界干扰
            if((ros::Time::now() - mission_last_time > ros::Duration(0.5)) && (loop_count > 0))
        	{
//                if((abs(current_pos.pose.position.x - pose_pub.pose.position.x) < 0.04) &&
//                   (abs(current_pos.pose.position.y - pose_pub.pose.position.y) < 0.04) &&
//                   (abs(current_pos.pose.position.z - pose_pub.pose.position.z) < 0.04))
                if(circle_distance(current_pos.pose.position.x,pose_pub.pose.position.x,
                                   current_pos.pose.position.y,pose_pub.pose.position.y,
                                   current_pos.pose.position.z,pose_pub.pose.position.z) < 0.05)
                {
                    acc_count++;
                    if(acc_count > 5)
                    {
                        current_mission_state = mission_num_hover_spray;
                        loop_count = 0;
                        acc_count = 0;
                        mission_last_time = ros::Time::now();
                        break;
                    }
                }
                else
                {
                    acc_count = 0;
                }
                /* count for max time */
                loop_count++;
                if(loop_count > 10)
                {
                    /* force spray */
                    current_mission_state = mission_num_hover_spray;
                    loop_count = 0;
                    acc_count = 0;
                    mission_last_time = ros::Time::now();
                    break;
                }
        		mission_last_time = ros::Time::now();
        		/* TODO: start spraying. -libn */
        	}
        break;
        case mission_num_hover_spray:
            pose_pub.pose.position.x = board10.drawingboard[current_mission_num].x - SPRAY_DISTANCE * cos(yaw_sp_calculated_m2p_data.yaw_sp);	/* TODO:switch to different board positions. -libn */
            pose_pub.pose.position.y = board10.drawingboard[current_mission_num].y - SPRAY_DISTANCE * sin(yaw_sp_calculated_m2p_data.yaw_sp);
            pose_pub.pose.position.z = board10.drawingboard[current_mission_num].z + SAFE_HEIGHT_DISTANCE;
            subtimer_start = false; //在喷涂阶段，不受子任务定时器的影响
            /* add height adjustment  --start. */
            if(ros::Time::now() - mission_last_time > ros::Duration(0.5))	/* spray for 5 seconds. -libn */
            {
                pose_pub.pose.position.x = board10.drawingboard[current_mission_num].x - SPRAY_DISTANCE * cos(yaw_sp_calculated_m2p_data.yaw_sp);	/* TODO:switch to different board positions. -libn */
                pose_pub.pose.position.y = board10.drawingboard[current_mission_num].y - SPRAY_DISTANCE * sin(yaw_sp_calculated_m2p_data.yaw_sp);
                pose_pub.pose.position.z = board10.drawingboard[current_mission_num].z + SAFE_HEIGHT_DISTANCE - 0.05f; //飞机下移
            }
            /* add height adjustment  --stop. */
            // 定时完成喷涂任务
            if(ros::Time::now() - mission_last_time > ros::Duration(3))
            {
                current_mission_state = mission_hover_after_stretch_back;
                mission_last_time = ros::Time::now();
                subtimer_start = true; //喷完后，开启子任务定时器
            }
        break;
        case mission_hover_after_stretch_back:
            pose_pub.pose.position.x = current_pos.pose.position.x; //没有位置环控制
            pose_pub.pose.position.y = current_pos.pose.position.y;
            pose_pub.pose.position.z = current_pos.pose.position.z;
            if(ros::Time::now() - mission_last_time > ros::Duration(0.5f))	/* spray for 5 seconds. -libn */
            {
                current_mission_state = mission_observe_point_go;
                current_mission_num = -1; //新的子任务，新的显示屏数字识别
            }
        break;
        case mission_num_done: //所有任务都已经完成，需要返航或者修复存在问题的子任务
            current_subtask_time = ros::Time::now(); //所有喷涂子任务完成的时间进行记录，作为超时修复任务的开始时刻
        	pose_pub.pose.position.x = current_pos.pose.position.x;	/* hover in current position. -libn */
        	pose_pub.pose.position.y = current_pos.pose.position.y;
        	pose_pub.pose.position.z = current_pos.pose.position.z;
        	/* TODO: mission check: if there are failures to be fixed -libn */
            if(FAILURE_REPAIR && mission_failure_acount != 0)
        	{
                current_mission_state = mission_fix_failure; //对问题子任务进行修复
                #ifdef NO_ROS_DEBUG
                    ROS_INFO("TODO: mission_fix_failure!");
                #endif
        	}
        	else			/* mission is finished. -libn */
        	{
                current_mission_state = mission_return_home; //不存在问题子任务，直接回home点
                #ifdef NO_ROS_DEBUG
                    ROS_INFO("going to mission_return_home");
                #endif
        		mission_last_time = ros::Time::now();
        	}
        break;
        case mission_fix_failure:
            /* TODO: add mission_failure_acount_fixed. -libn */
            if((ros::Time::now() < mission_timer_start_time + ros::Duration(MAX_FLIGHT_TIME)) &&
                (mission_failure_acount != 0))
            {
                current_mission_num = failure[mission_failure_acount-1].num; //出现问题的子任务数字
                /* deal with failures. */
                // 判断那些任务可以修复，哪些不能修复
                if((mission_num_search <= failure[mission_failure_acount-1].state &&
                    failure[mission_failure_acount-1].state <= mission_arm_spread) ||
                   failure[mission_failure_acount-1].state == mission_hover_before_spary)
                {
                    current_mission_state = mission_num_search; //如果有错误，将从loop==6开始开始弥补
                                                                //在错误处理过程中只会受到总时间的制约，一直进行此错误的修复
                }
                mission_failure_acount--;
            }
            else if(mission_failure_acount == 0)
            {
                #ifdef NO_ROS_DEBUG
                    ROS_INFO("All failure fixed, return to home.");
                #endif
                current_mission_state = mission_return_home;
            }
        break;
        case mission_force_return_home: //强制返航，在总任务时间超时的情况下会出现
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
                #ifdef NO_ROS_DEBUG
                    ROS_INFO("start mission_hover_only");
                #endif
                current_mission_state = mission_hover_only;
				mission_last_time = ros::Time::now();
			}
			break;
        case mission_hover_only:
			pose_pub.pose.position.x = setpoint_H.pose.position.x;
			pose_pub.pose.position.y = setpoint_H.pose.position.y;
			pose_pub.pose.position.z = setpoint_H.pose.position.z;
            if(ros::Time::now() - mission_last_time > ros::Duration(1))	/* hover for 1 seconds. -libn */
			{
                current_mission_state = land;
			}
			break;
        case land:
			break;
    }
}

// ！！！！存在的问题！！！！：
// 1. 宏定义ROS_RATE决定了大循环的运行频率，可以考虑使用该值
// √ 2. current_mission_num的最初默认值设置为0,不太合适。虽然后面给赋值成为-1了
// √ 3. 子任务超时，要记录故障任务状态，同时也应该判断current_mission_num是否更新，这会影响后面的补救策略
// √ 4. 返回当前显示屏数字时，会保持上一loop的识别数字
// 5. 起飞旋停后，没有x、y方向的位置控制
// 6. 喷涂阶段出现预扫时，右点旋停后会跳至状态mission_num_locate，左点旋停后会跳至状态mission_num_search，需要统一设置为mission_num_search
// √ 7. 关于可修复子任务的判断，可以更完善一些
// 8. 子任务修复时，也可以引入时间的约束
