# School project - Robotic Bartender

# Pre-requisites: 
- Ubuntu 18.04 + ROS Melodic
- dynamixel_sdk
- dynamixel_workbench
- moveit!
- open_manipulator_msgs
- rviz-visual-tools

Assuming you already have a catkin workspace "catkin_ws" in your home directory: 
```
cd ~/catkin_ws/src
git clone https://github.com/KeeJin/Bartending-Robot.git --recursive-submodules
git checkout devel
sudo apt-get install ros-melodic-dynamixel-sdk
sudo apt install ros-melodic-rviz-visual-tools
sudo apt install ros-melodic-moveit
cd ..
catkin_make
```