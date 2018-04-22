#include <ros/ros.h>
#include <stdio.h> //sprintf, FILE* operations
#include <iostream>
#include <vector>
#include "string.h"
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

class SimController
{
private:
	ros::Subscriber state_sub;
	ros::Publisher local_pos_pub;
	ros::ServiceClient arming_client, set_mode_client;

	geometry_msgs::PoseStamped pose;
	ros::Time last_request;
	mavros_msgs::SetMode offb_set_mode;
	mavros_msgs::CommandBool arm_cmd;

	int m_group_index;
	mavros_msgs::State current_state;

	void stateCallback(const mavros_msgs::State::ConstPtr& msg)
	{
		current_state = *msg;
	}

public:
	SimController(const ros::NodeHandle& n)
	{
		ros::NodeHandle nh("~");//~ means private param
		nh.getParam("group_index", m_group_index);

		char msg_name[50];
		sprintf(msg_name,"/uav%d/mavros/state", m_group_index);
		state_sub = nh.subscribe<mavros_msgs::State>(msg_name,10,&SimController::stateCallback,this);
		
		sprintf(msg_name,"/uav%d/mavros/setpoint_position/local", m_group_index);
		local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>(msg_name,10);
		
		sprintf(msg_name,"/uav%d/mavros/cmd/arming", m_group_index);
		arming_client = nh.serviceClient<mavros_msgs::CommandBool>(msg_name);
		
		sprintf(msg_name,"/uav%d/mavros/set_mode", m_group_index);
		set_mode_client = nh.serviceClient<mavros_msgs::SetMode>(msg_name);

		//note:att_est,raw_ctrl_sp,pos_ctrl_sp,IMU data might be needed and they can be added here


	}
	~SimController(){}

	void run(double frequency)
	{
		ros::NodeHandle node;
		ros::Rate rate(20.0);
		while(ros::ok() && !current_state.connected)
		{
			ros::spinOnce();
			rate.sleep();
		}
		ROS_INFO("uav%d successfully connected~~~", m_group_index);

		pose.pose.position.x = 0;
		pose.pose.position.y = 0;
		pose.pose.position.z = 2;

		for (int i = 100; i > 0 && ros::ok(); --i)
		{
			local_pos_pub.publish(pose);
			ros::spinOnce();
			rate.sleep();
		}
		
		offb_set_mode.request.custom_mode = "OFFBOARD";
		arm_cmd.request.value = true;
		last_request = ros::Time::now();

		ros::Timer timer = node.createTimer(ros::Duration(1.0/frequency), &SimController::iteration,this);
		ros::spin();
	}

	void iteration(const ros::TimerEvent& e)
	{
		//ROS_INFO("iteration~~~");
		if (current_state.mode != "OFFBOARD" &&	(ros::Time::now()-last_request > ros::Duration(5.0)))
		{
			if (set_mode_client.call(offb_set_mode) &&
				offb_set_mode.response.mode_sent)
			{
				ROS_INFO("Offboard enabled");
			}
		}else{
			if (!current_state.armed && (ros::Time::now()-last_request>ros::Duration(5.0)))
			{
				if (arming_client.call(arm_cmd) && arm_cmd.response.success)
				{
					ROS_INFO("Vehicle armed");
				}
				last_request = ros::Time::now();
			}
		}

		local_pos_pub.publish(pose);
	}


	
};

int main(int argc, char **argv)
{
	ros::init(argc,argv,"sim_controller");
	ros::NodeHandle n;

	SimController sc(n);
	sc.run(50);

	return 0;
}