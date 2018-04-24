#include "ros/ros.h"
#include <stdio.h> //sprintf
#include <iostream>
#include <vector>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/UInt8.h>
#include "Eigen/Eigen/Eigen"
#include "Eigen/Eigen/Geometry"

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

using namespace Eigen;
using namespace std;
using namespace cv;

int m_vehicle_number = 5;

class SimCommander
{
private:
	std::vector<ros::Publisher> pos_sp_pub_v;
	std::vector<ros::Subscriber> flight_mode_sub_v;

	Mat laplacian_matrix;
	float step;
	std::vector<geometry_msgs::PoseStamped> pos_ctrl_sp, current_pos, target_pos, relative_pos;
	std::vector<bool> hadRecieved;

	enum FlightMode {
	    TakingOff  = 1,
	    Commanding = 2,
	    Hovering = 3,
  	};
  	std::vector<FlightMode> m_flight_mode;
  	bool enableCommanding;

  	void flightModeCallback(const std_msgs::UInt8::ConstPtr& msg, int vehicle_index)
	{		
		int value = int(msg->data);
		m_flight_mode[vehicle_index] = FlightMode(value);
		ROS_INFO("~~~~~~~~~~uav%d m_flight_mode : %d \n", vehicle_index, value);
	}

	void positionCallback(const geometry_msgs::PoseStamped::ConstPtr& msg, int vehicle_index)
	{
		if (!hadRecieved[vehicle_index])
			hadRecieved[vehicle_index] = true;
		// ROS_INFO("uav%d had recieved its local position", vehicle_index);
		current_pos[vehicle_index] = *msg;
	}

public:
	SimCommander(ros::NodeHandle& nh)
	:pos_sp_pub_v(m_vehicle_number)
	,flight_mode_sub_v(m_vehicle_number)
	,m_flight_mode(m_vehicle_number)
	,pos_ctrl_sp(m_vehicle_number)
	,current_pos(m_vehicle_number)
	,target_pos(m_vehicle_number)
	,relative_pos(m_vehicle_number)
	,hadRecieved(m_vehicle_number)
	,enableCommanding(false)
	,step(0.2f)
	{
		char msg_name[50];
		for(int i=0;i<m_vehicle_number;i++){
			m_flight_mode[i] = TakingOff;

			sprintf(msg_name,"/uav%d/swarm/setpoint_position",i);
			pos_sp_pub_v[i] = nh.advertise<geometry_msgs::PoseStamped>(msg_name, 1);
			
			// sprintf(msg_name,"/uav%d/swarm/flight_mode",i);
			// flight_mode_sub_v[i] = nh.subscribe<std_msgs::UInt8>(msg_name,10,boost::bind(&SimCommander::flightModeCallback, this, _1, i));
		
			sprintf(msg_name,"/uav%d/mavros/local_position/pose",i);
			flight_mode_sub_v[i] = nh.subscribe<geometry_msgs::PoseStamped>(msg_name,10,boost::bind(&SimCommander::positionCallback, this, _1, i));
			
			hadRecieved[i] = false;
			target_pos[i].pose.position.z = 2;
		}	

		target_pos[0].pose.position.x = 0.0;
		target_pos[0].pose.position.y = 0.0;
		target_pos[1].pose.position.x = 0.618;
		target_pos[1].pose.position.y = -1.902;
		target_pos[2].pose.position.x = 2.618;
		target_pos[2].pose.position.y = -1.902;
		target_pos[3].pose.position.x = 3.236;
		target_pos[3].pose.position.y = 0.0;
		target_pos[4].pose.position.x = 1.618;
		target_pos[4].pose.position.y = 1.176;

		laplacian_matrix = Mat::zeros(m_vehicle_number, m_vehicle_number, CV_32SC1);
		laplacian_matrix_AllConnect(m_vehicle_number);
	}

	void run(double frequency)
	{
		ros::NodeHandle node;
		ROS_INFO("commander start running~~~~");
		// ros::Rate loop_rate(10);
		// while(ros::ok() && !enableCommanding)
		// {
		// 	int tmp_count = 0;
		// 	for (int i = 0; i < m_vehicle_number; ++i)
		// 	{
		// 		if (m_flight_mode[i] == FlightMode(2))
		// 		{
		// 			tmp_count++;
		// 		}
		// 	}
		// 	if (tmp_count == m_vehicle_number)
		// 	{
		// 		enableCommanding = true;
		// 	} else
		// 		ROS_INFO("Shit!!!! %d", tmp_count);
		// 	ros::spinOnce();
		// 	loop_rate.sleep();
		// }
		// ROS_INFO("enable Commanding~~~~");
		ros::Timer timer = node.createTimer(ros::Duration(1.0/frequency), &SimCommander::iteration, this);
		ros::spin();
	}
	void iteration(const ros::TimerEvent& e)
	{	

		/*prepare : target position matrix, current positions, relative target position vector, Laplacian matrix*/
		for (int i = 0; i < m_vehicle_number; ++i)
		{
			relative_pos[i].pose.position.x = current_pos[i].pose.position.x - target_pos[i].pose.position.x;
			relative_pos[i].pose.position.y = current_pos[i].pose.position.y - target_pos[i].pose.position.y;
		}
		/*obtain position control setpoint*/
		std::vector<Vector2f> pos_sp_t(m_vehicle_number);
		for (int i = 0; i < m_vehicle_number; ++i)
		{
			pos_sp_t[i](0) = 0.0;
			pos_sp_t[i](1) = 0.0;
			for (int j = 0; j < m_vehicle_number; ++j)
			{
				pos_sp_t[i](0) += laplacian_matrix.at<int>(i,j)*relative_pos[i].pose.position.x;
				pos_sp_t[i](1) += laplacian_matrix.at<int>(i,j)*relative_pos[i].pose.position.y;
			}
		}

		for (int i = 0; i < m_vehicle_number; ++i)
		{
			pos_ctrl_sp[i].pose.position.x = -pos_sp_t[i](0)*step + current_pos[i].pose.position.x;
			pos_ctrl_sp[i].pose.position.y = -pos_sp_t[i](1)*step + current_pos[i].pose.position.y;
			pos_ctrl_sp[i].pose.position.z = 2;
			pos_sp_pub_v[i].publish(pos_ctrl_sp[i]);
		}

	}

/***********************
Concensus contol of UAVs
***********************/
	void laplacian_matrix_AllConnect(int n) 
	{
		for(int i=0;i<m_vehicle_number;++i)
		{
			for(int j=0;j<m_vehicle_number;++j)
			{
				if(i!=j)
				{
					laplacian_matrix.at<int>(i,j) = -1;
				}else{
					laplacian_matrix.at<int>(i,j)= m_vehicle_number-1;	
				}
			}
		}
	}
};

int main(int argc, char **argv)
{
//  int ret = init_scan(argc, argv);
	//glutInit(&argc, argv);
	ros::init(argc, argv, "sim_commander");
	ros::NodeHandle n;

	// int vehicle_num;
	// n.getParam("/vehicle_number", vehicle_num);
	SimCommander sc(n);
	sc.run(50);


  return 0;


}
