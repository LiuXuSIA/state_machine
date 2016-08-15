/**
* @file     : state_machine.cpp
* @brief    : state_machine: 1) get 4 setpoints; 2) takeoff; 3) get 10 drawing boards' positions; 4) land.
* @author   : libn
* @time     : Aug 11, 2016 9:45:22 AM
*/
#include "ros/ros.h"
#include "state_machine/Attitude.h"
#include "state_machine/State.h"
#include <geometry_msgs/PoseStamped.h>  /* message type of /mavros/local_position/pose (P.S. It is included in dir: /opt/ros/indigo/share/geometry_msgs/msg) -libn */
#include "state_machine/ActuatorControl.h" /* add actuator_control output */
#include <stdio.h>
#include <state_machine/CommandTOL.h>	/* head file for takeoff&land-command service -libn */
#include "state_machine/Setpoint.h"
#include "state_machine/DrawingBoard.h"

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

state_machine::Setpoint setpoint_indexed;
void printSetpointIndexedCallback(const state_machine::Setpoint::ConstPtr& msg)
{
//    ROS_INFO("I heard: [%f] [%f] [%f]",msg->x, msg->y, msg->z);
	setpoint_indexed = *msg;
}

state_machine::DrawingBoard board;
void DrawingBoardCallback(const state_machine::DrawingBoard::ConstPtr& msg)
{
//    ROS_INFO("I heard: [%f] [%f] [%f]",msg->x, msg->y, msg->z);
	board = *msg;
}

/* 4 setpoints. -libn <Aug 15, 2016 11:04:14 AM> */
geometry_msgs::PoseStamped setpoint_A;
geometry_msgs::PoseStamped setpoint_B;
geometry_msgs::PoseStamped setpoint_C;
geometry_msgs::PoseStamped setpoint_D;

/* 10 drawing board position. -libn <Aug 15, 2016 11:20:52 AM> */
state_machine::DrawingBoard board_1;
state_machine::DrawingBoard board_2;
state_machine::DrawingBoard board_3;
state_machine::DrawingBoard board_4;
state_machine::DrawingBoard board_5;
state_machine::DrawingBoard board_6;
state_machine::DrawingBoard board_7;
state_machine::DrawingBoard board_8;
state_machine::DrawingBoard board_9;
state_machine::DrawingBoard board_0;

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

// position setpoint to publish
geometry_msgs::PoseStamped setpoint_pub;
// pulisher: used to publish local_pos_setpoint -libn
ros::Publisher local_pos_setpoint_pub;

int count;	/* to reduce display freq. -libn <Aug 15, 2016 10:02:32 PM> */
int main(int argc, char **argv)
{
	ros::init(argc, argv, "state_machine");

	ros::NodeHandle nh;

	ROS_INFO("setpoints suscribe start!");

	/* receive indexed setpoint. -libn <Aug 15, 2016 9:08:05 AM> */
	ros::Subscriber setpoint_Indexed_sub = nh.subscribe("Setpoint_Indexed", 100 ,printSetpointIndexedCallback);

	/* receive 10 drawing board position. -libn <Aug 15, 2016 11:06:00 AM> */
	ros::Subscriber drawingboard_Indexed_sub = nh.subscribe("DrawingBoard_Position", 100 ,DrawingBoardCallback);

	ros::Rate rate(10);		/* 10Hz. -libn <Aug 11, 2016 9:28:08 AM> */

	// get pixhawk's status
	ros::Subscriber state_sub = nh.subscribe<state_machine::State>("mavros/state", 10, state_cb);

	// get pixhawk's local position
	ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, pos_cb);

	/* publish local_pos_setpoint -libn <Aug 11, 2016 10:05:05 AM> */
	local_pos_setpoint_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);

	ros::Time landing_last_request = ros::Time::now();

    // takeoff and land service
    // ros::ServiceClient takeoff_client = nh.serviceClient<state_machine::CommandTOL>("mavros/cmd/takeoff");
    ros::ServiceClient land_client = nh.serviceClient<state_machine::CommandTOL>("mavros/cmd/land");

    // wait for FCU connection
	while(ros::ok() && !current_state.connected){
		ros::spinOnce();
		rate.sleep();
	}

    setpoint_pub.pose.position.x = 0;
    setpoint_pub.pose.position.y = 0;
    setpoint_pub.pose.position.z = 3;

	//send a few setpoints before starting  --> for safety
    ROS_INFO("sending 100 desired_poses. Wait for a while.");
	for(int i = 100; ros::ok() && i > 0; --i){
		local_pos_setpoint_pub.publish(setpoint_pub);
		ros::spinOnce();
		rate.sleep();
	}

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

        /* get 4 setpoints:A,B,C,D. -libn <Aug 15, 2016 9:55:15 AM> */
        switch(setpoint_indexed.index)
        {
        	case 1:
        		setpoint_A.pose.position.x = setpoint_indexed.x;
				setpoint_A.pose.position.y = setpoint_indexed.y;
				setpoint_A.pose.position.z = setpoint_indexed.z;
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


		/* get 10 drawing board's letter and position. -libn <Aug 15, 2016 11:09:44 AM> */
		switch(board.num)
		{
			case 0:
				board_0.num = 0;
				board_0.x = board.x;
				board_0.y = board.y;
				board_0.z = board.z;
				break;
			case 1:
				board_1.num = 1;
				board_1.x = board.x;
				board_1.y = board.y;
				board_1.z = board.z;
				break;
			case 2:
				board_2.num = 2;
				board_2.x = board.x;
				board_2.y = board.y;
				board_2.z = board.z;
				break;
			case 3:
				board_3.num = 3;
				board_3.x = board.x;
				board_3.y = board.y;
				board_3.z = board.z;
				break;
			case 4:
				board_4.num = 4;
				board_4.x = board.x;
				board_4.y = board.y;
				board_4.z = board.z;
				break;
			case 5:
				board_5.num = 5;
				board_5.x = board.x;
				board_5.y = board.y;
				board_5.z = board.z;
				break;
			case 6:
				board_6.num = 6;
				board_6.x = board.x;
				board_6.y = board.y;
				board_6.z = board.z;
				break;
			case 7:
				board_7.num = 7;
				board_7.x = board.x;
				board_7.y = board.y;
				board_7.z = board.z;
				break;
			case 8:
				board_8.num = 8;
				board_8.x = board.x;
				board_8.y = board.y;
				board_8.z = board.z;
				break;
			case 9:
				board_9.num = 9;
				board_9.x = board.x;
				board_9.y = board.y;
				board_9.z = board.z;
				break;
			default:
				ROS_INFO("setpoint index error!");
				break;
		}

		// auto task off
        if( current_state.mode == "AUTO.TAKEOFF"){
            ROS_INFO("AUTO TAKEOFF!");
        }

        state_machine_func();	/* Run state_machine. -libn <Aug 11, 2016 10:01:12 AM> */

    	count++;
    	if(count >= 20)
    	{
    		count = 0;
    		ROS_INFO("current_pos_state: %d",current_pos_state);
    	}

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
            setpoint_pub = setpoint_A;			// set expected position
        	local_pos_setpoint_pub.publish(setpoint_pub);      // publish pos A (desired position)
            if((abs(current_pos.pose.position.x - setpoint_A.pose.position.x) < 0.2) &&      // switch to next state
               (abs(current_pos.pose.position.y - setpoint_A.pose.position.y) < 0.2) &&
               (abs(current_pos.pose.position.z - setpoint_A.pose.position.z) < 0.2))
               {
                    current_pos_state = POS_B; // current_pos_state++;
               }
            break;
        case POS_B:
            setpoint_pub = setpoint_B;			// set expected position
        	local_pos_setpoint_pub.publish(setpoint_pub);      // publish pos B (desired position)
            if((abs(current_pos.pose.position.x - setpoint_B.pose.position.x) < 0.2) &&      // switch to next state
               (abs(current_pos.pose.position.y - setpoint_B.pose.position.y) < 0.2) &&
               (abs(current_pos.pose.position.z - setpoint_B.pose.position.z) < 0.2))
               {
            	current_pos_state = POS_C; // current_pos_state++;
               }
            break;
        case POS_C:
            setpoint_pub = setpoint_C;			// set expected position
        	local_pos_setpoint_pub.publish(setpoint_pub);      // publish pos C (desired position)
			if((abs(current_pos.pose.position.x - setpoint_C.pose.position.x) < 0.2) &&      // switch to next state
			   (abs(current_pos.pose.position.y - setpoint_C.pose.position.y) < 0.2) &&
			   (abs(current_pos.pose.position.z - setpoint_C.pose.position.z) < 0.2))
			   {
					current_pos_state = POS_D; // current_pos_state++;
			   }
			break;
        case POS_D:
            setpoint_pub = setpoint_D;			// set expected position
        	local_pos_setpoint_pub.publish(setpoint_pub);      // publish pos D (desired position)
			if((abs(current_pos.pose.position.x - setpoint_D.pose.position.x) < 0.2) &&      // switch to next state
			   (abs(current_pos.pose.position.y - setpoint_D.pose.position.y) < 0.2) &&
			   (abs(current_pos.pose.position.z - setpoint_D.pose.position.z) < 0.2))
			   {
				current_pos_state = LAND; // current_pos_state++;
			   }
			break;
        case LAND:
            break;
    }

}
