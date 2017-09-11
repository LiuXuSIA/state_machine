/**
* @file       : state_machine_demo.cpp
* @brief      : take off --> pos A --> hover --> pos B --> hover --> pos A --> land
                a demo for state machine
* @author     : liu zhong
* @time       : 2017/07/04
*/

#include <ros/ros.h>
#include <math.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <state_machine/CommandTOL.h>
#include <state_machine/State.h>

void state_machine_func(void);
bool Shown = false; //用于显示的标志位

// 定义变量，代表状态机状态
static const int NONE = 0;
static const int TAKE_OFF = 1;
static const int POS_A = 2;
static const int A_HOVER = 3;
static const int POS_B = 4;
static const int B_HOVER = 5;
static const int POS_C = 6;
static const int C_HOVER = 7;
static const int POS_A_Re = 8;
static const int LAND = 9;
// 当前的状态机状态
int current_pos_state = NONE;

/************** 发布 **************/
// 设置起飞期望速度
geometry_msgs::TwistStamped vel_take_off;
ros::Publisher vel_take_off_pub;
// 设置A、B点位置变量，将通过ros发送出去
geometry_msgs::PoseStamped pose_a;
geometry_msgs::PoseStamped pose_b;
geometry_msgs::PoseStamped pose_c;
ros::Publisher local_pos_pub;
// 配置ros service，用于降落
state_machine::CommandTOL landing_cmd;
ros::ServiceClient tol_client;

/************** 订阅 **************/
// 订阅“状态”消息，以及相应回调函数
state_machine::State current_state;
void state_cb(const state_machine::State::ConstPtr& msg){
    current_state = *msg;
}
ros::Subscriber state_sub;
// 订阅“位置”消息，以及相应的回调函数
geometry_msgs::PoseStamped current_pos;
void pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_pos = *msg;
}
ros::Subscriber pos_sub;

/************** 时间戳 **************/
ros::Time last_request;

// ----- 主函数 ----- //
int main(int argc, char **argv)
{
    ros::init(argc, argv, "state_machine_demo"); //节点名称
    ros::NodeHandle nh; //ros节点句柄
    ros::Rate rate(10.0); // 设置循环频率

    // --初始化变量--
    // 起飞的期望速度
    vel_take_off.twist.linear.z = 2; //速度跟踪，保持航向角
    // A点位置，东北天
    pose_a.pose.position.x = 0;
    pose_a.pose.position.y = 0;
    pose_a.pose.position.z = 5;
    // B点位置
    pose_b.pose.position.x = 0;
    pose_b.pose.position.y = 5;
    pose_b.pose.position.z = 5;
    // C点位置
    pose_c.pose.position.x = -5;
    pose_c.pose.position.y = 5;
    pose_c.pose.position.z = 5;
    // 姿态期望，主要是航向
    float yaw_sp=M_PI_2;
    pose_a.pose.orientation.x = 0; //机头将指向北
    pose_a.pose.orientation.y = 0;
    pose_a.pose.orientation.z = sin(yaw_sp/2);
    pose_a.pose.orientation.w = cos(yaw_sp/2);
    pose_b.pose.orientation.x = 0;
    pose_b.pose.orientation.y = 0;
    pose_b.pose.orientation.z = sin(yaw_sp/2);
    pose_b.pose.orientation.w = cos(yaw_sp/2);
    pose_c.pose.orientation.x = 0;
    pose_c.pose.orientation.y = 0;
    pose_c.pose.orientation.z = sin(yaw_sp/2);
    pose_c.pose.orientation.w = cos(yaw_sp/2);
    // 降落参数设置landing_cmd
    landing_cmd.request.min_pitch = 1.0; //降落保持航向角

    // 用于消息发布
    vel_take_off_pub = nh.advertise<geometry_msgs::TwistStamped>("mavros/setpoint_velocity/cmd_vel", 10);
    local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    tol_client = nh.serviceClient<state_machine::CommandTOL>("mavros/cmd/land");
    // 用于消息订阅
    state_sub = nh.subscribe<state_machine::State>("mavros/state", 10, state_cb);
    pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, pos_cb);
    // 时间戳
    last_request = ros::Time::now();

    // 等待飞行控制器链接
    while(ros::ok() && !current_state.connected){
        // 如果控制器没连接上，将一直处于循环里
        ROS_INFO("Waiting connection.");
        ros::spinOnce();
        rate.sleep();
    }

    // 开始之前先要进行期望位置的发送！！！
    for(int i = 50; ros::ok() && i > 0; --i){
        ROS_INFO("Sending msg before starting.");
        local_pos_pub.publish(pose_a);
        ros::spinOnce();
        rate.sleep();
    }
    last_request = ros::Time::now();

    while(ros::ok()){
        // 状态机修改飞行器状态
        state_machine_func();

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}

// ----- 任务状态机 ----- //
void state_machine_func(void)
{
    switch(current_pos_state)
    {
        case NONE:
            // 这样保证在解锁过程中一直有位置数据发送
            current_pos_state=TAKE_OFF;
            ROS_INFO("Start state machine!");
        break;
        case TAKE_OFF:
            if(Shown == false)
            {
                ROS_INFO("Taking off!");
                Shown = true;
            }
            // 设定期望起飞速度
            vel_take_off_pub.publish(vel_take_off);
            if(current_pos.pose.position.z > 3)
            {
                current_pos_state = POS_A;
                Shown = false;
            }
        break;
        case POS_A:
            if(Shown == false)
            {
                ROS_INFO("Flying to position A!");
                Shown = true;
            }
            local_pos_pub.publish(pose_a);
            if(abs(current_pos.pose.position.x - pose_a.pose.position.x) < 0.1 && //到达A点后悬停
               abs(current_pos.pose.position.y - pose_a.pose.position.y) < 0.1 &&
               abs(current_pos.pose.position.z - pose_a.pose.position.z) < 0.1)
            {
                current_pos_state = A_HOVER;
                last_request = ros::Time::now();
                Shown = false;
            }
        break;
        case A_HOVER:
            if(Shown == false)
            {
                ROS_INFO("Hovering at position A!");
                Shown = true;
            }
            local_pos_pub.publish(pose_a);
            if(ros::Time::now() - last_request > ros::Duration(5.0)) //旋停5s
            {
                current_pos_state = POS_B;
                Shown = false;
            }
        break;
        case POS_B:
            if(Shown == false)
            {
                ROS_INFO("Flying to position B!");
                Shown = true;
            }
            local_pos_pub.publish(pose_b);
            if(abs(current_pos.pose.position.x - pose_b.pose.position.x) < 0.1 && //到达B点后悬停
               abs(current_pos.pose.position.y - pose_b.pose.position.y) < 0.1 &&
               abs(current_pos.pose.position.z - pose_b.pose.position.z) < 0.1)
            {
                current_pos_state = B_HOVER;
                last_request = ros::Time::now();
                Shown = false;
            }
        break;
        case B_HOVER:
            if(Shown == false)
            {
                ROS_INFO("Hovering at position B!");
                Shown = true;
            }
            local_pos_pub.publish(pose_b);
            if(ros::Time::now() - last_request > ros::Duration(5.0))
            {
                current_pos_state = POS_C;
                Shown = false;
            }
        break;
        case POS_C:
            if(Shown == false)
            {
                ROS_INFO("Flying to position C!");
                Shown = true;
            }
            local_pos_pub.publish(pose_c);
            if(abs(current_pos.pose.position.x - pose_c.pose.position.x) < 0.1 && //到达B点后悬停
               abs(current_pos.pose.position.y - pose_c.pose.position.y) < 0.1 &&
               abs(current_pos.pose.position.z - pose_c.pose.position.z) < 0.1)
            {
                current_pos_state = C_HOVER;
                last_request = ros::Time::now();
                Shown = false;
            }
        break;
        case C_HOVER:
            if(Shown == false)
            {
                ROS_INFO("Hovering at position C!");
                Shown = true;
            }
            local_pos_pub.publish(pose_c);
            if(ros::Time::now() - last_request > ros::Duration(5.0))
            {
                current_pos_state = POS_A_Re;
                Shown = false;
            }
        break;
        case POS_A_Re:
            if(Shown == false)
            {
                ROS_INFO("Going back to position A!");
                Shown = true;
            }
            local_pos_pub.publish(pose_a);
            if(abs(current_pos.pose.position.x - pose_a.pose.position.x) < 0.1 && //回到A点后降落
               abs(current_pos.pose.position.y - pose_a.pose.position.y) < 0.1 &&
               abs(current_pos.pose.position.z - pose_a.pose.position.z) < 0.1)
            {
                current_pos_state = LAND;
                last_request = ros::Time::now();
                Shown = false;
            }
        break;
        case LAND:
            if(current_state.mode == "OFFBOARD" &&
               current_state.mode != "AUTO.LAND" &&
               ros::Time::now() - last_request > ros::Duration(1.0))
            {
                if(tol_client.call(landing_cmd) &&
                    landing_cmd.response.success)
                {
                    if(Shown == false)
                    {
                        ROS_INFO("AUTO LANDING!");
                        Shown = true;
                    }
                    last_request = ros::Time::now();
                }
            }
        break;
    }
}
