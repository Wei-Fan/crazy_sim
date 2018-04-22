# crazy_sim is a simulation workplace of crazyflie swarm

# Run it in Gazebo:
cd src/SIMULATION/Firmware/
source ~/catkin_ws/devel/setup.bash    // (optional)
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/posix_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo
roslaunch px4 multi_uav_mavros_sitl.launch
# open another terminal
roslaunch crazy_sim test.launch

# Run one vehicle:
cd src/SIMULATION/Firmware/
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/posix_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo
make posix_sitl_default gazebo
# open a new terminal
roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557" 
roslaunch mavros px4.launch fcu_url:="udp://:14540@192.168.1.36:14557"
