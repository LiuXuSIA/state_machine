/**
* @file     : state_machine.cpp
* @brief    :
* @author   : libn
* @time     : Aug 11, 2016 9:45:22 AM
*/
#include "ros/ros.h"
#include "state_machine/Attitude.h"
#include "state_machine/Setpoint.h"		/* to send 4 setpoints -libn */
#include "state_machine/State.h"
#include <geometry_msgs/PoseStamped.h>  /* message type of /mavros/local_position/pose (P.S. It is included in dir: /opt/ros/indigo/share/geometry_msgs/msg) -libn */
#include "state_machine/ActuatorControl.h" /* add actuator_control output */
#include <stdio.h>
#include <state_machine/CommandTOL.h>	/* head file for takeoff&land-command service -libn */

void state_machine_func(void);

// state machine's states
static const int POS_A = 0;
static const int POS_B = 1;
static const int POS_C = 2;
static const int POS_D = 3;
static const int LAND  = 4;
static const int TAKEOFF  = 5;	/* TODO! for future use. -libn <Aug 11, 2016 9:54:13 AM> */

// current positon state, init state is takeoff and go to setpoint_A
int current_pos_state = POS_A;

// position setpoint to publish
geometry_msgs::PoseStamped setpoint_pub;

// pulisher: used to publish local_pos_setpoint -libn
ros::Publisher local_pos_setpoint_pub;

state_machine::Setpoint setpoint_A;
state_machine::Setpoint setpoint_B;
state_machine::Setpoint setpoint_C;
state_machine::Setpoint setpoint_D;
void printSetpointACallback(const state_machine::Setpoint::ConstPtr& msg)
{
//    ROS_INFO("I heard: [%f] [%f] [%f]",msg->x, msg->y, msg->z);
    setpoint_A = *msg;
}
void printSetpointBCallback(const state_machine::Setpoint::ConstPtr& msg)
{
//    ROS_INFO("I heard: [%f] [%f] [%f]",msg->x, msg->y, msg->z);
    setpoint_B = *msg;
}
void printSetpointCCallback(const state_machine::Setpoint::ConstPtr& msg)
{
//    ROS_INFO("I heard: [%f] [%f] [%f]",msg->x, msg->y, msg->z);
    setpoint_C = *msg;
}
void printSetpointDCallback(const state_machine::Setpoint::ConstPtr& msg)
{
//    ROS_INFO("I heard: [%f] [%f] [%f]",msg->x, msg->y, msg->z);
    setpoint_D = *msg;
}


// local position msg callback function
geometry_msgs::PoseStamped current_pos;
void pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_pos = *msg;
}

// state msg callback function
state_machine::State current_state;
void state_cb(const state_machine::State::ConstPtr& msg){
    current_state = *msg;
}

/*
void printATTCallback(const state_machine::Attitude::ConstPtr& msg)
{
	ROS_INFO("I heard:[%f] [%f] [%f]",msg->roll, msg->pitch, msg->yaw);
}


void printSTATECallback(const state_machine::State::ConstPtr& msg)
{
	printf("I heard:[%d] [%d] [%d]\n",msg->connected, msg->armed, msg->guided);	
	//printf("I heard:[%d] [%d] [%d] [%s]\n",msg->connected, msg->armed, msg->guided, msg->mode);
	//ROS_INFO("I heard:[%d] [%d] [%d] [%s]",msg->connected, msg->armed, msg->guided, msg->mode);
}
*/
int main(int argc, char **argv)
{
	ros::init(argc, argv, "state_machine");

	ros::NodeHandle nh;
/*
//	ros::Subscriber ATT_sub = nh.subscribe("mavros/attitude", 10 ,printATTCallback);
//	ros::Subscriber STATE_sub = nh.subscribe("mavros/state", 10 ,printSTATECallback);

//	geometry_msgs::PoseStamped pose;
//	pose.pose.position.x = 0;
//	pose.pose.position.y = 0;
//	pose.pose.position.z = 2;
//	ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);

//	state_machine::ActuatorControl output;
//	//output.				//TODO!!
//	//output.controls[1] = 0;
//	ros::Publisher acturtor_pub = nh.advertise<state_machine::ActuatorControl>("mavros/actuator_control", 10);

*/

	/* Step 1: receive 4 setpoints. -libn <Aug 10, 2016 2:57:25 PM> */
	ros::Subscriber setpoint_A_sub = nh.subscribe("Setpoint_A", 100 ,printSetpointACallback);
	ros::Subscriber setpoint_B_sub = nh.subscribe("Setpoint_B", 100 ,printSetpointBCallback);
	ros::Subscriber setpoint_C_sub = nh.subscribe("Setpoint_C", 100 ,printSetpointCCallback);
	ros::Subscriber setpoint_D_sub = nh.subscribe("Setpoint_D", 100 ,printSetpointDCallback);
	ROS_INFO("setpoints suscribe start!");

	ros::Rate rate(10);		/* 10Hz. -libn <Aug 11, 2016 9:28:08 AM> */

	// get pixhawk's local position
	ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, pos_cb);

	/* publish local_pos_setpoint -libn <Aug 11, 2016 10:05:05 AM> */
	ros::Publisher local_pos_setpoint_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
	//send a few setpoints before starting  --> for safety
	for(int i = 100; ros::ok() && i > 0; --i){
		setpoint_pub.pose.position.x = setpoint_A.x;			// set expected position
		setpoint_pub.pose.position.y = setpoint_A.y;			// set expected position
		setpoint_pub.pose.position.z = setpoint_A.z;			// set expected position
		local_pos_setpoint_pub.publish(setpoint_pub);      // publish pos A (desired position)
		ros::spinOnce();
		rate.sleep();
	}

    ros::Time landing_last_request = ros::Time::now();

    // takeoff and land service
    // ros::ServiceClient takeoff_client = nh.serviceClient<state_machine::CommandTOL>("mavros/cmd/takeoff");
    ros::ServiceClient land_client = nh.serviceClient<state_machine::CommandTOL>("mavros/cmd/land");

    // takeoff and landing
    // mavros_msgs::CommandTOL takeoff_cmd;
    // takeoff_cmd.request.min_pitch = -1.0;
    state_machine::CommandTOL landing_cmd;
    landing_cmd.request.min_pitch = 1.0;

	while(ros::ok())
	{
		/* TODO	takeoff */
        // auto task off
        if( current_state.mode == "AUTO.TAKEOFF"){
            ROS_INFO("AUTO TAKEOFF!");
        }

		ROS_INFO("setpoint_A received: [%f] [%f] [%f]",setpoint_A.x, setpoint_A.y, setpoint_A.z);
		ROS_INFO("setpoint_B received: [%f] [%f] [%f]",setpoint_B.x, setpoint_B.y, setpoint_B.z);
		ROS_INFO("setpoint_C received: [%f] [%f] [%f]",setpoint_C.x, setpoint_C.y, setpoint_C.z);
		ROS_INFO("setpoint_D received: [%f] [%f] [%f]",setpoint_D.x, setpoint_D.y, setpoint_D.z);

		// auto task off
        if( current_state.mode == "AUTO.TAKEOFF"){
            ROS_INFO("AUTO TAKEOFF!");
        }

        state_machine_func();	/* Run state_machine. -libn <Aug 11, 2016 10:01:12 AM> */

        // landing
        if(current_pos_state == LAND){

            if( current_state.mode != "AUTO.LAND" &&
            (ros::Time::now() - landing_last_request > ros::Duration(5.0))){
            if(land_client.call(landing_cmd) &&
                landing_cmd.response.success){
                ROS_INFO("AUTO LANDING!");
            }
            landing_last_request = ros::Time::now();
            }
        }

		ros::spinOnce();	/* refresh data subscriber. -libn <Aug 11, 2016 9:28:33 AM> */
		rate.sleep();

	}


	return 0;
		
}


// task state machine
void state_machine_func(void)
{
    switch(current_pos_state){

    	case TAKEOFF:	/* Not used! just in case.  -libn <Aug 11, 2016 9:56:44 AM> */
    		break;

        case POS_A:
        	setpoint_pub.pose.position.x = setpoint_A.x;			// set expected position
        	setpoint_pub.pose.position.y = setpoint_A.y;			// set expected position
        	setpoint_pub.pose.position.z = setpoint_A.z;			// set expected position
//        	local_pos_setpoint_pub.publish(setpoint_pub);      // publish pos A (desired position) //problem!! TODO! -20160811
//        	local_pos_setpoint_pub.publish(pose_a);      // first publish pos A (desired position)
            if((abs(current_pos.pose.position.x - setpoint_A.x) < 0.2) &&      // switch to next state
               (abs(current_pos.pose.position.y - setpoint_A.y) < 0.2) &&
               (abs(current_pos.pose.position.z - setpoint_A.z) < 0.2))
               {
                    current_pos_state = POS_B; // current_pos_state++;
               }
            break;
        case POS_B:
        	setpoint_pub.pose.position.x = setpoint_B.x;			// set expected position
        	setpoint_pub.pose.position.y = setpoint_B.y;			// set expected position
        	setpoint_pub.pose.position.z = setpoint_B.z;			// set expected position
//        	local_pos_setpoint_pub.publish(setpoint_pub);      // publish pos B (desired position)
//            local_pos_pub.publish(pose_b);      // first publish pos B (desired position)
            if((abs(current_pos.pose.position.x - setpoint_B.x) < 0.2) &&      // switch to next state
               (abs(current_pos.pose.position.y - setpoint_B.y) < 0.2) &&
               (abs(current_pos.pose.position.z - setpoint_B.z) < 0.2))
               {
            	current_pos_state = POS_C; // current_pos_state++;
               }
            break;
        case POS_C:
        	setpoint_pub.pose.position.x = setpoint_C.x;			// set expected position
        	setpoint_pub.pose.position.y = setpoint_C.y;			// set expected position
        	setpoint_pub.pose.position.z = setpoint_C.z;			// set expected position
//        	local_pos_setpoint_pub.publish(setpoint_pub);      // publish pos A (desired position)
//            local_pos_pub.publish(pose_a);      // first publish pos C (desired position)
			if((abs(current_pos.pose.position.x - setpoint_C.x) < 0.2) &&      // switch to next state
			   (abs(current_pos.pose.position.y - setpoint_C.y) < 0.2) &&
			   (abs(current_pos.pose.position.z - setpoint_C.z) < 0.2))
			   {
					current_pos_state = POS_D; // current_pos_state++;
			   }
			break;
        case POS_D:
        	setpoint_pub.pose.position.x = setpoint_D.x;			// set expected position
        	setpoint_pub.pose.position.y = setpoint_D.y;			// set expected position
        	setpoint_pub.pose.position.z = setpoint_D.z;			// set expected position
//        	local_pos_setpoint_pub.publish(setpoint_pub);      // publish pos A (desired position)
//            local_pos_pub.publish(pose_a);      // first publish pos D (desired position)
			if((abs(current_pos.pose.position.x - setpoint_D.x) < 0.2) &&      // switch to next state
			   (abs(current_pos.pose.position.y - setpoint_D.y) < 0.2) &&
			   (abs(current_pos.pose.position.z - setpoint_D.z) < 0.2))
			   {
				current_pos_state = LAND; // current_pos_state++;
			   }
			break;
        case LAND:
            break;
    }
}
