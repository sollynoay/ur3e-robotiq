# UR3e-ROBOTIQ
This repository is for ROS setup for real UR3e arm and ROBOTIQ gripper (2F-85)
# Download the project
```
git clone https://github.com/sollynoay/ur3e-robotiq.git
cd src
git clone https://github.com/sollynoay/universal_robot.git
git clone https://github.com/sollynoay/Universal_Robots_ROS_Driver.git
git clone https://github.com/sollynoay/robotiq_2finger_grippers.git
```
# UR3e Setup
Here, controlling UR3e by ROS is explained. UR3e can be connected to PC via ethernet.  
## ip and mask for PC and UR3e control box
This work sets PC to 192.168.0.180 and UR3e to 192.168.0.179, masks to 255.255.255.0. This ensures communication under TCP/IP.  
(Note: the UR3e teaching panel can be connected with a keyboard. You can stop GUI and use CUI mode. Username: root, password: easybot. This seems not necessary but FYI.)
## Calibration
If this is the first time or the calibration file cannot be found, run the following code to read the calibration parameters from the robot. 
```
roslaunch ur_calibration calibration_correction.launch robot_ip:=192.168.0.179 target_filename:="${HOME}/my_robot_calibration.yaml"
```
## Start Service
```
roslaunch ur_robot_driver ur3e_bringup.launch robot_ip:=192.168.0.179 kinematics_config:="${HOME}/my_robot_calibration.yaml"
```
## Setup on UR3e teaching panel
Now, it is necessary to add external control to program in the teaching panel of UR robot. The external control is possible by installing URCap to teaching panel, externalcontrol-1.0.5.urcap. To start it, you need to add external control to program tree in the teaching panel and run it. Then, you can control the robot arm with MoveIt.

# ROBOTIQ setup
To connect the gripper to PC, it is also necessary to install URCap to UR3e teaching panel, rs485-1.0.urcap. Note: after installing rs485-1.0.urcap, it is necessary to uninstall the previous Robotiq urcap. This means robotiq can no longer be controlled by the teaching panel. To begin with, map the /dev/ttyTool in UR control box to your PC.  
To make sure you have permission to connect from PC,
```
sudo chmod -R 777 /dev
```
Then, map /dev/ttyTool to your PC  
```
roscore
python ./tool_test.py
```
## Start the service
```
roslaunch robotiq_2f_gripper_control test_85mm_gripper_new.launch
```
It can now listen to the command to open or close the gripper. For instance, you can run as an example,
```
rosrun robotiq_2f_gripper_control robotiq_2f_action_close.py 
```
# MoveIt for UR3e and ROBOTIQ
MoveIt config is generated in ./src/moveit_test  
After starting UR3e and ROBOTIQ, run the following command to start move_group
```
roslaunch moveit_test move_group.launch
```
Then, you can generate plans and execute them with MoveIt.  
For visualization
```
roslaunch moveit_test moveit_rviz.launch
```
## MoveIt python interface
```
rosrun moveit_python_interface move.py
```

# UR3e without MoveIt
You can also directly send commands to UR3e without using MoveIt.
```
rosrun ur_robot_driver test_move
```
You have several controllers to choose:
```
Available trajectory controllers:
0 (joint-based): scaled_pos_joint_traj_controller
1 (joint-based): scaled_vel_joint_traj_controller
2 (joint-based): pos_joint_traj_controller
3 (joint-based): vel_joint_traj_controller
4 (joint-based): forward_joint_traj_controller
5 (Cartesian): pose_based_cartesian_traj_controller
6 (Cartesian): joint_based_cartesian_traj_controller
7 (Cartesian): forward_cartesian_traj_controller
Please choose a controller by entering its number (Enter '0' if you are unsure / don't care): 0
```
Note that in ur3e_bringup.launch, scaled_pos_joint_traj_controller is started by default. 

# Trouble Shooting
```
CMake Error at CMakeLists.txt:1:
  Parse error.  Expected a command name, got unquoted argument with text
  "/opt/ros/noetic/share/catkin/cmake/toplevel.cmake"
```
Delete the CMakeList in src.  
```
 Could NOT find moveit_core (missing: moveit_core_DIR)
```






