#include "ros/ros.h"
#include "easyfly/Swarm_Add.h"
#include "easyfly/LogBlock.h"
#include <stdio.h> //sprintf
int g_vehicle_num;
int main(int argc, char **argv)
{
  ros::init(argc, argv, "crazyflie_add", ros::init_options::AnonymousName);
  ros::NodeHandle n("~");
  
  std::string uri;
  std::string  tf_prefix;
  float roll_trim;
  float pitch_trim;
  int group_index;

  bool enable_logging;
  bool enable_logging_imu;
  bool enable_parameters;
  bool use_ros_time;
  bool enable_logging_att;
  //int g_joy_num;

  n.getParam("g_vehicle_num",g_vehicle_num);
  n.getParam("group_index", group_index);
  n.getParam("uri", uri);
  n.getParam("tf_prefix", tf_prefix);
  n.getParam("roll_trim", roll_trim);
  n.getParam("pitch_trim", pitch_trim);

  n.param("enable_logging", enable_logging,true);
  n.param("enable_logging_imu", enable_logging_imu,true);
  n.param("enable_parameters", enable_parameters,true);

  n.param("use_ros_time", use_ros_time,true);
  n.param("enable_logging_battery", enable_logging_att,true);

//publish the number of vehicles
  ros::NodeHandle n_numv;
  char msg_name[50];

  ROS_INFO("wait_for_service add_crazyflie");
  char servicename[50];
  ros::ServiceClient addCrazyflieService = n.serviceClient<easyfly::Swarm_Add>("/add_crazyflie"); //client instance
  addCrazyflieService.waitForExistence();
  ROS_INFO("found%s","/add_crazyflie");
  
    easyfly::Swarm_Add addCrazyflie; //containt of req and res

    addCrazyflie.request.g_vehicle_num = g_vehicle_num;

    addCrazyflie.request.group_index = group_index;
    addCrazyflie.request.uri = uri;
    addCrazyflie.request.tf_prefix = tf_prefix;
    addCrazyflie.request.roll_trim = roll_trim;
    addCrazyflie.request.pitch_trim = pitch_trim;
    addCrazyflie.request.enable_logging = enable_logging;
    addCrazyflie.request.enable_logging_imu = enable_logging_imu;
    addCrazyflie.request.enable_parameters = enable_parameters;
    addCrazyflie.request.use_ros_time = use_ros_time;
    addCrazyflie.request.enable_logging_att = enable_logging_att;

  std::vector<std::string> genericLogTopics;
  n.param("genericLogTopics", genericLogTopics, std::vector<std::string>());
  std::vector<int> genericLogTopicFrequencies;
  n.param("genericLogTopicFrequencies", genericLogTopicFrequencies, std::vector<int>()); 
  
  if (genericLogTopics.size() == genericLogTopicFrequencies.size())
  {

    size_t i = 0;
    for (auto& topic : genericLogTopics)
    {
      easyfly::LogBlock logBlock;
      logBlock.topic_name = topic;
      logBlock.frequency = genericLogTopicFrequencies[i];
      n.getParam("genericLogTopic_" + topic + "_Variables", logBlock.variables);
      addCrazyflie.request.log_blocks.push_back(logBlock);
      ++i;
    }
  }
  else
  {
    ROS_ERROR("Cardinality of genericLogTopics and genericLogTopicFrequencies does not match!");
  }

  addCrazyflieService.call(addCrazyflie);
  
  return 0;
}
