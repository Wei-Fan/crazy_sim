#include <ros/ros.h>
#include <stdio.h> //sprintf, FILE* operations
#include <iostream>
#include <vector>
#include "string.h"
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/UInt8.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

class SimController
{
private:
	ros::Subscriber state_sub, pos_sp_sub;
	ros::Publisher local_pos_pub, flight_mode_pub;
	ros::ServiceClient arming_client, set_mode_client;

	geometry_msgs::PoseStamped pose,pos_ctrl_sp;
	ros::Time last_request;
	mavros_msgs::SetMode offb_set_mode;
	mavros_msgs::CommandBool arm_cmd;

	int m_group_index;
	mavros_msgs::State current_state;

	enum FlightMode {
	    TakingOff  = 1,
	    Commanding = 2,
	    Hovering = 3,
  	}m_flight_mode;

	void stateCallback(const mavros_msgs::State::ConstPtr& msg)
	{
		current_state = *msg;
	}

	void positionSetPointCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
	{
		pos_ctrl_sp = *msg;
	}

public:
	SimController()
	:m_flight_mode(TakingOff)
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

		sprintf(msg_name,"/uav%d/swarm/flight_mode", m_group_index);
		flight_mode_pub = nh.advertise<std_msgs::UInt8>(msg_name,10);
		for (int i = 0; i < 100; ++i)
		{
			flight_mode_pub.publish(int(m_flight_mode));
		}

		sprintf(msg_name,"/uav%d/swarm/setpoint_position", m_group_index);
		pos_sp_sub = nh.subscribe<geometry_msgs::PoseStamped>(msg_name,10,&SimController::positionSetPointCallback,this);
		
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

		// ROS_INFO("uav%d ~~~~~~ 1", m_group_index);
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
		/*taking off*/
		if (current_state.mode != "OFFBOARD" &&	(ros::Time::now()-last_request > ros::Duration(3.0)))
		{	

			// ROS_INFO("uav%d ~~~~~~~~~ 1", m_group_index);
			if (set_mode_client.call(offb_set_mode) &&
				offb_set_mode.response.mode_sent)
			{
				ROS_INFO("uav%d Offboard enabled", m_group_index);
			}
		}else if (!current_state.armed && (ros::Time::now()-last_request>ros::Duration(3.0))){
			// ROS_INFO("uav%d ~~~~~~~~~~~ 3", m_group_index);
			if (arming_client.call(arm_cmd) && arm_cmd.response.success)
			{
				ROS_INFO("uav%d Vehicle armed", m_group_index);
			}
			last_request = ros::Time::now();
		}else if (current_state.armed && (ros::Time::now()-last_request>ros::Duration(6.0)))
		{
			m_flight_mode = Commanding;
			flight_mode_pub.publish(int(m_flight_mode));
		}

		/*controller*/
		if (m_flight_mode == Commanding)
		{
			//ROS_INFO("uav%d Commanding", m_group_index);
			//pose = pos_ctrl_sp;
		}
		local_pos_pub.publish(pose);
	}


	
};

int main(int argc, char **argv)
{
	ros::init(argc,argv,"sim_controller");
	ros::NodeHandle n;

	SimController sc;
	sc.run(50);

	return 0;
}