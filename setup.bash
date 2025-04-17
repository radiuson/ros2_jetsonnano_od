source /opt/ros/foxy/setup.bash
source /home/jetson/code/cv_bridge_ws/install/setup.bash
source /home/jetson/code/dofbot_ros2_ws/install/setup.bash
export AMENT_PREFIX_PATH=/home/jetson/code/dofbot_ros2_ws/install/ros2_vision_arm_control:$AMENT_PREFIX_PATH
export PYTHONPATH=$PYTHONPATH:/home/jetson/code/dofbot_ros2_ws/install/ros2_vision_arm_control/lib/python3.8/site-packages
export ROS_DOMAIN_ID=10
export ROS_HOSTNAME=10.42.0.215