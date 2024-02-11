# UR3e-ROBOTIQ
This repository is for ROS setup for real ur3e arm and robotiq gripper
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
(Note: the UR3e teaching panel can be connected with a key board. You can stop GUI and use terminal mode. Username: root, password: easybot. This seems not necessary but FYI.)
## Calibration
If for the first time or calibration is required.
```
roslaunch ur_calibration calibration_correction.launch robot_ip:=192.168.0.179 target_filename:="${HOME}/my_robot_calibration.yaml"
```
## Start
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
It can now listen to the command to open or close the gripper. For instance, you can run,
```
rosrun robotiq_2f_gripper_control robotiq_2f_action_close.py 
```





