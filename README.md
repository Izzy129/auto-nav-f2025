This is shared repo for Robotics AutoNav Team. 


### Resources
## GNSS
Simulating odometry in Gazebo:
https://roboticsknowledgebase.com/wiki/common-platforms/ros/ros-mapping-localization/#:~:text=Then%20you%20can%20simply%20launch,publishes%20nav_msgs%2FOdometry%20type%20messages.

## GNSS stimulation with Gazebo 
To use, in WSL terminal do the following after git cloning:
cd ~/auto-nav-f2025/Testing_gazebo_ros2_bridge
colcon build
source install/setup.bash
ros2 launch ros_gz_example_bringup diff_drive.launch.py

Gazebo and RViz should show up with an example rover. 
