#include "ros/ros.h"
#include <stdio.h> //sprintf
#include <iostream>
#include <vector>

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

	MatrixXf m_Connection_Matrix;

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
		// ROS_INFO("~~~uav%d m_flight_mode : %d \n", vehicle_index, value);
	}

public:
	SimCommander(ros::NodeHandle& nh)
	:pos_sp_pub_v(m_vehicle_number)
	,flight_mode_sub_v(m_vehicle_number)
	,m_flight_mode(m_vehicle_number)
	,enableCommanding(false)
	{
		char msg_name[50];
		for(int i=0;i<m_vehicle_number;i++){
			m_flight_mode[i] = TakingOff;

			sprintf(msg_name,"/uav%d/swarm/setpoint_position",i);
			pos_sp_pub_v[i] = nh.advertise<geometry_msgs::PoseStamped>(msg_name, 1);
			
			sprintf(msg_name,"/uav%d/swarm/flight_mode",i);
			flight_mode_sub_v[i] = nh.subscribe<std_msgs::UInt8>(msg_name,5,boost::bind(&SimCommander::flightModeCallback, this, _1, i));
		}		
	}

	void run(double frequency)
	{
		ros::NodeHandle node;

		while(!enableCommanding)
		{
			int tmp_count = 0;
			for (int i = 0; i < m_vehicle_number; ++i)
			{
				if (m_flight_mode[i] == Commanding)
				{
					tmp_count++;
				}
			}
			if (tmp_count == m_vehicle_number)
			{
				enableCommanding = true;
			}
		}

		ros::Timer timer = node.createTimer(ros::Duration(1.0/frequency), &SimCommander::iteration, this);
		ros::spin();
	}
	void iteration(const ros::TimerEvent& e)
	{	

		// static float time_elapse = 0;
		// _dt_deriv = e.current_real.toSec() - e.last_real.toSec();
		// time_elapse += _dt_deriv;

		// //cut off
		// if(m_joy_v[0].curr_buttons[4] == 1 && m_joy_v[0].curr_buttons[5] == 1){
		// 	m_cmd_msg.cut = 1;
		// }
		// else{
		// 	switch(m_flight_mode){
		// 		case MODE_RAW:{
					
		// 			for(int i=0; i<m_vehicle_number;i++){
		// 				m_ctrl_v[i].rawctrl_msg.raw_att_sp.x = -m_joy_v[0].axes[0] * 30 * DEG2RAD;//+-1
		// 				m_ctrl_v[i].rawctrl_msg.raw_att_sp.y = m_joy_v[0].axes[1] * 30 * DEG2RAD;
		// 				m_ctrl_v[i].rawctrl_msg.raw_att_sp.z = m_joy_v[0].axes[2] * 20 * DEG2RAD;//rate
		// 				m_ctrl_v[i].rawctrl_msg.throttle = m_joy_v[0].axes[3];//0-1
		// 				if(m_ctrl_v[i].rawctrl_msg.throttle<0){
		// 					m_ctrl_v[i].rawctrl_msg.throttle=0;
		// 				}
		// 				m_rawpub_v[i].publish(m_ctrl_v[i].rawctrl_msg);
		// 			}
		// 		}
		// 		break;
		// 		case MODE_POS:{

		// 			if(m_joy_v[0].changed_arrow[1] == false){ 
		// 				//faire rien..
		// 			}

		// 			if(m_joy_v[0].changed_arrow[1] == true && m_joy_v[0].curr_arrow[1] == 1){//take off
		// 				m_joy_v[0].changed_arrow[1] = false;
		// 				if(m_flight_state == Idle){
		// 					for(int i=0;i<m_vehicle_number;i++){
		// 						posspReset(i,&swarm_pos);
		// 						yawspReset(i);
		// 					}
		// 					m_flight_state = TakingOff;
		// 					//printf("hello!!!!!run\n");
		// 				}
		// 			}	
		// 			else if(m_joy_v[0].changed_arrow[1] == true && m_joy_v[0].curr_arrow[1] == -1){//land
		// 				printf("landing!!\n");
		// 				m_joy_v[0].changed_arrow[1] = false;
		// 				if(m_flight_state == Hovering)
		// 				{
		// 					m_flight_state = Landing;
		// 				}
		// 			}
					
		// 			switch(m_flight_state){
		// 				case Idle:{
		// 					for(int i=0;i<m_vehicle_number;i++){
		// 					}
		// 					//all motors off
		// 					break;
		// 				}
						
		// 				case Automatic:{
		// 					for(int i=0;i<m_vehicle_number;i++){
		// 						float pos_move_rate[3];
		// 						pos_move_rate[0] = m_joy_v[i].axes[0] * 2.0;
		// 						pos_move_rate[1] = m_joy_v[i].axes[1] * 2.0;
		// 						pos_move_rate[2] = m_joy_v[i].axes[3] * 1.0;
		// 						m_ctrl_v[i].posctrl_msg.pos_sp.x += pos_move_rate[0];
		// 						m_ctrl_v[i].posctrl_msg.pos_sp.y += pos_move_rate[1];
		// 						m_ctrl_v[i].posctrl_msg.pos_sp.z += pos_move_rate[2];
		// 						m_ctrl_v[i].posctrl_msg.vel_ff.x = pos_move_rate[0];
		// 						m_ctrl_v[i].posctrl_msg.vel_ff.y = pos_move_rate[1];
		// 						m_ctrl_v[i].posctrl_msg.vel_ff.z = pos_move_rate[2];
		// 						float yaw_move_rate = m_joy_v[i].axes[2] * 20 * DEG2RAD;
		// 						m_ctrl_v[i].posctrl_msg.yaw_sp += yaw_move_rate;
		// 						//m_pospub_v[i].publish(m_ctrl_v[i].posctrl_msg);

		// 					}
		// 					break;
		// 				}
						
		// 				case TakingOff:{			
		// 					//printf("takeof!\n");				
		// 					if(!isFirstVicon && isGotAtt)
		// 					{
		// 						int count_hovering_num = 0;
		// 						for(int i=0;i<m_vehicle_number;i++){
		// 							command_takeoff(i,_dt_deriv);
		// 							if(swarm_pos[i](2) > takeoff_objective_height - m_takeoff_switch_Hover)// + m_takeoff_switch_Hover)
	 //    							{
	 //    								++count_hovering_num;
	 //    							}
	 //    							if(count_hovering_num == m_vehicle_number)
	 //    							{
	 //    								isHovering = true;
	 //    								m_flight_state = Hovering;
	 //    							}
		// 						}
		// 					}
		// 					break;
		// 				}
		// 				case Landing:{
		// 					for(int i=0;i<m_vehicle_number;i++){
		// 						control_landing(i, _dt_deriv);
		// 					}
		// 					break;
		// 				}
						
		// 				case Hovering:
		// 				{	
		// 					if(m_hovering_time>=10.0f)
		// 					{
		// 						printf("####Already hover over 10 second!!!###\n");
		// 						m_flight_state = Circling;//test circle control
		// 						m_hovering_time = 0.0f;
		// 					}
		// 					ros::Time begin_hovering = ros::Time::now();
		// 					for(int i=0;i<m_vehicle_number;++i)
		// 					{	
		// 						posspReset(i, &m_hover_pos);
		// 					}
		// 					ros::Time after_hovering = ros::Time::now();
		// 					ros::Duration d = after_hovering - begin_hovering;
		// 					m_hovering_time += _dt_deriv;
		// 					break;
		// 				}
						
		// 				case Circling:{
		// 					printf("begin circling!!\n");
		// 					for(int i=0;i<m_vehicle_number;i++){
		// 						command_circling(_dt_deriv);
		// 					}
		// 					break;
		// 				}

		// 				case Funny:{
		// 					break;
		// 				}
		// 			}//end switch state
		// 			for(int i=0;i<m_vehicle_number;++i)
		// 			{
		// 				m_pospub_v[i].publish(m_ctrl_v[i].posctrl_msg);
		// 			}
		// 		}//end case posctrl mode
		// 		break;
		// 		case MODE_TRJ:{
					
		// 		}
		// 		break;
		// 		default:
		// 		break;
		// 	} //end switch mode
		// }//end of cut off case
		// m_cmd_msg.flight_state = m_flight_state;
		// m_cmdpub.publish(m_cmd_msg);
		// m_cmd_msg.l_flight_state = m_flight_state;
	}

/***********************
Concensus contol of UAVs
***********************/
	void fill_AjMatrix_AllConnect(float* weight) 
	{
		for(int i=0;i<m_vehicle_number;++i)
		{
			for(int j=0;j<m_vehicle_number;++j)
			{
				if(i!=j)
				{
					m_Connection_Matrix(i,j) = *weight;
				}
				else{
					m_Connection_Matrix(i,j) = 0;	
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
