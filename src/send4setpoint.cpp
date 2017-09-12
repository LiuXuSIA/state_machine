/**
* @file     : send4setpoint.cpp
* @brief    : sent 4 setpoints(A,B,C,D)
* @author   : libn
* @time     : Aug 12, 2016 3:57:08 PM
*/
/* 以10Hz的频率发送4个虚拟的位置点 */
#include "ros/ros.h"
#include <stdio.h>
#include <math.h>

#include <state_machine/Setpoint.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "send4setpoints");
    ros::NodeHandle nh;
    ros::Rate loop_rate(10);

    /* send indexed setpoint. -libn <Aug 15, 2016 9:00:02 AM> */
    state_machine::Setpoint setpoint_indexed;
    ros::Publisher setpoint_indexed_pub = nh.advertise<state_machine::Setpoint>("Setpoint_Indexed", 10);
    setpoint_indexed.index = 1;	/* 1st setpoint. -libn <Aug 15, 2016 9:01:21 AM> */
    setpoint_indexed.x = 0.0f;
    setpoint_indexed.y = 0.0f;	/* ROS coordinate frame: ENU(East/North/Up) -libn */
    setpoint_indexed.z = 0.0f;

	while(ros::ok())
	{
        setpoint_indexed.index = 1;	/* 1st setpoint(A). -libn <Aug 15, 2016 9:01:21 AM> */ //setpoint(O)，观察点
        setpoint_indexed.x = 0.0f;  /* ROS coordinate frame: ENU(East/North/Up) -libn */
        setpoint_indexed.y = 5.0f;
        setpoint_indexed.z = 4.0f;  /* not necessary. */
		setpoint_indexed_pub.publish(setpoint_indexed);
        ros::spinOnce();   //ROS消息回调处理函数，作为检测消息回调函数的触发，并且还会继续运行下面的代码
                           //注意：ros::spin()与上面函数的作用类似，但是不会继续运行下面的代码
        loop_rate.sleep(); //消耗时间

        setpoint_indexed.index = 2;	/* 2ed setpoint(L). -libn <Aug 15, 2016 9:01:21 AM> */
		setpoint_indexed.x = 0.0f;
		setpoint_indexed.y = 2.0f;
        setpoint_indexed.z = 5.0f;  /* not necessary. */
		setpoint_indexed_pub.publish(setpoint_indexed);
		ros::spinOnce();
		loop_rate.sleep();

        setpoint_indexed.index = 3;	/* 3rd setpoint(R). -libn <Aug 15, 2016 9:01:21 AM> */
        setpoint_indexed.x = 2.0f;
		setpoint_indexed.y = 2.0f;
        setpoint_indexed.z = 5.0f;  /* not necessary. */
		setpoint_indexed_pub.publish(setpoint_indexed);
		ros::spinOnce();
		loop_rate.sleep();

        /* 4th setpoint(D): not necessary. */
		setpoint_indexed.index = 4;	/* 4th setpoint(D). -libn <Aug 15, 2016 9:01:21 AM> */
		setpoint_indexed.x = -2.0f;
		setpoint_indexed.y = 0.0f;
		setpoint_indexed.z = 5.0f;
		setpoint_indexed_pub.publish(setpoint_indexed);
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
