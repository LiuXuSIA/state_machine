 /**
* @file     : offb_simulation_test.cpp
* @brief    : offboard simulation test: demo rewritten -> 4 setpoints flight -> complete state machine.
* @author   : libn
* @Time     : Aug 25, 201610:06:42 PM
*/

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include "state_machine/CommandBool.h"
#include "state_machine/SetMode.h"
#include "state_machine/State.h"
#include "state_machine/CommandTOL.h"
#include "state_machine/Setpoint.h"

void state_machine_func(void);
// state machine's states
static const int POS_A = 1;
static const int POS_B = 2;
static const int POS_C = 3;
static const int POS_D = 4;
static const int POS_H = 5;
static const int LAND  = 6;
static const int TAKEOFF  = 0;	/* TODO! for future use. -libn */
// current positon state, init state is takeoff and go to setpoint_A
int current_pos_state = POS_A;

state_machine::State current_state;
void state_cb(const state_machine::State::ConstPtr& msg){
    current_state = *msg;
}

state_machine::Setpoint setpoint_indexed;
void SetpointIndexedCallback(const state_machine::Setpoint::ConstPtr& msg)
{
	setpoint_indexed = *msg;
}

// local position msg callback function
geometry_msgs::PoseStamped current_pos;
void pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    current_pos = *msg;
}

/* 4 setpoints. -libn */
geometry_msgs::PoseStamped setpoint_A;
geometry_msgs::PoseStamped setpoint_B;
geometry_msgs::PoseStamped setpoint_C;
geometry_msgs::PoseStamped setpoint_D;
geometry_msgs::PoseStamped setpoint_H;	/* home position == setpoint_A. -libn */

geometry_msgs::PoseStamped pose_pub;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<state_machine::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
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

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(10.0);

    // wait for FCU connection
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }


    pose_pub.pose.position.x = 0;
    pose_pub.pose.position.y = 0;
    pose_pub.pose.position.z = 2;
    pose_pub.pose.orientation.x = 0;			/* orientation expressed using quaternion. -libn */
	pose_pub.pose.orientation.y = 0;			/* w = cos(theta/2), x = nx * sin(theta/2),  y = ny * sin(theta/2), z = nz * sin(theta/2) -libn */
	pose_pub.pose.orientation.z = 0.707;
	pose_pub.pose.orientation.w = 0.707;		/* set yaw* = 90 degree(default in simulation). -libn */

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose_pub);
        ros::spinOnce();
        rate.sleep();
    }

    state_machine::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    state_machine::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();
    ros::Time t = ros::Time::now();
    ros::Time last_show_request = ros::Time::now();

    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" && current_state.mode != "AUTO.LAND" &&	/* set offboard mode for the first time. -libn */
          (ros::Time::now() - last_request > ros::Duration(5.0)))
        {
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.success){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed && current_state.mode != "AUTO.LAND" &&	/* set armed for the first time. -libn */
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                    t = ros::Time::now();
                }
                last_request = ros::Time::now();
            }

        }

		// landing
		if(current_state.armed && ros::Time::now() - t > ros::Duration(10.0) && current_pos_state == LAND)	/* set landing mode until uav stopped. -libn */
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

		/* get 4 setpoints:A,B,C,D. -libn */
		switch(setpoint_indexed.index)
		{
			case 1:
				setpoint_A.pose.position.x = setpoint_indexed.x;
				setpoint_A.pose.position.y = setpoint_indexed.y;
				setpoint_A.pose.position.z = setpoint_indexed.z;
				setpoint_H.pose.position.x = setpoint_indexed.x;
				setpoint_H.pose.position.y = setpoint_indexed.y;
				setpoint_H.pose.position.z = setpoint_indexed.z;
				break;
			case 2:
				setpoint_B.pose.position.x = setpoint_indexed.x;
				setpoint_B.pose.position.y = setpoint_indexed.y;
				setpoint_B.pose.position.z = setpoint_indexed.z;
				break;
			case 3:
				setpoint_C.pose.position.x = setpoint_indexed.x;
				setpoint_C.pose.position.y = setpoint_indexed.y;
				setpoint_C.pose.position.z = setpoint_indexed.z;
				break;
			case 4:
				setpoint_D.pose.position.x = setpoint_indexed.x;
				setpoint_D.pose.position.y = setpoint_indexed.y;
				setpoint_D.pose.position.z = setpoint_indexed.z;
				break;
			default:
				ROS_INFO("setpoint index error!");
				break;
		}

		if(current_state.mode == "OFFBOARD" && current_state.armed)
		{
			state_machine_func();
		}

		if(current_state.mode == "OFFBOARD" && current_state.armed && ros::Time::now() - last_show_request > ros::Duration(0.5))	/* set message display delay. -libn */
		{
			ROS_INFO("setpoint_received:\n"
					"setpoint_A:%5.3f %5.3f %5.3f \n"
					"setpoint_B:%5.3f %5.3f %5.3f \n"
					"setpoint_C:%5.3f %5.3f %5.3f \n"
					"setpoint_D:%5.3f %5.3f %5.3f \n"
					"setpoint_H:%5.3f %5.3f %5.3f",
					setpoint_A.pose.position.x,setpoint_A.pose.position.y,setpoint_A.pose.position.z,
					setpoint_B.pose.position.x,setpoint_B.pose.position.y,setpoint_B.pose.position.z,
					setpoint_C.pose.position.x,setpoint_C.pose.position.y,setpoint_C.pose.position.z,
					setpoint_D.pose.position.x,setpoint_D.pose.position.y,setpoint_D.pose.position.z,
					setpoint_H.pose.position.x,setpoint_H.pose.position.y,setpoint_H.pose.position.z);
			ROS_INFO("current position: %5.3f %5.3f %5.3f",current_pos.pose.position.x,current_pos.pose.position.y,current_pos.pose.position.z);
			ROS_INFO("position*: %5.3f %5.3f %5.3f\n",pose_pub.pose.position.x,pose_pub.pose.position.y,pose_pub.pose.position.z);
			ROS_INFO("current_pos_state: %d",current_pos_state);
			last_show_request = ros::Time::now();
		}

        local_pos_pub.publish(pose_pub);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

/* task state machine. -libn */
void state_machine_func(void)
{
	switch(current_pos_state)
	{
		case TAKEOFF:	/* Not used! just in case.  -libn <Aug 11, 2016 9:56:44 AM> */
    		break;

        case POS_A:
        	pose_pub.pose.position.x = setpoint_A.pose.position.x;
        	pose_pub.pose.position.y = setpoint_A.pose.position.y;
        	pose_pub.pose.position.z = setpoint_A.pose.position.z;
            if((abs(current_pos.pose.position.x - setpoint_A.pose.position.x) < 0.2) &&      // switch to next state
               (abs(current_pos.pose.position.y - setpoint_A.pose.position.y) < 0.2) &&
               (abs(current_pos.pose.position.z - setpoint_A.pose.position.z) < 0.2))
               {
                    current_pos_state = POS_B; // current_pos_state++;
               }
            break;
        case POS_B:
        	pose_pub.pose.position.x = setpoint_B.pose.position.x;
			pose_pub.pose.position.y = setpoint_B.pose.position.y;
			pose_pub.pose.position.z = setpoint_B.pose.position.z;
            if((abs(current_pos.pose.position.x - setpoint_B.pose.position.x) < 0.2) &&      // switch to next state
               (abs(current_pos.pose.position.y - setpoint_B.pose.position.y) < 0.2) &&
               (abs(current_pos.pose.position.z - setpoint_B.pose.position.z) < 0.2))
               {
            	current_pos_state = POS_C; // current_pos_state++;
               }
            break;
        case POS_C:
        	pose_pub.pose.position.x = setpoint_C.pose.position.x;
			pose_pub.pose.position.y = setpoint_C.pose.position.y;
			pose_pub.pose.position.z = setpoint_C.pose.position.z;
			if((abs(current_pos.pose.position.x - setpoint_C.pose.position.x) < 0.2) &&      // switch to next state
			   (abs(current_pos.pose.position.y - setpoint_C.pose.position.y) < 0.2) &&
			   (abs(current_pos.pose.position.z - setpoint_C.pose.position.z) < 0.2))
			   {
					current_pos_state = POS_D; // current_pos_state++;
			   }
			break;
        case POS_D:
        	pose_pub.pose.position.x = setpoint_D.pose.position.x;
			pose_pub.pose.position.y = setpoint_D.pose.position.y;
			pose_pub.pose.position.z = setpoint_D.pose.position.z;
			if((abs(current_pos.pose.position.x - setpoint_D.pose.position.x) < 0.2) &&      // switch to next state
			   (abs(current_pos.pose.position.y - setpoint_D.pose.position.y) < 0.2) &&
			   (abs(current_pos.pose.position.z - setpoint_D.pose.position.z) < 0.2))
			   {
				current_pos_state = POS_H; // current_pos_state++;
			   }
			break;
        case POS_H:
			pose_pub.pose.position.x = setpoint_H.pose.position.x;
			pose_pub.pose.position.y = setpoint_H.pose.position.y;
			pose_pub.pose.position.z = setpoint_H.pose.position.z;
			if((abs(current_pos.pose.position.x - setpoint_H.pose.position.x) < 0.2) &&      // switch to next state
			   (abs(current_pos.pose.position.y - setpoint_H.pose.position.y) < 0.2) &&
			   (abs(current_pos.pose.position.z - setpoint_H.pose.position.z) < 0.2))
			   {
				current_pos_state = LAND; // current_pos_state++;
			   }
			break;
        case LAND:
			break;
    }

}
