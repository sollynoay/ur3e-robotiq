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
First, it is necessary to set up the ip and mask for PC and UR3e control box. This work sets PC to 192.168.0.180 and UR3e to 192.168.0.179, masks to 255.255.255.0. This ensures communication under TCP/IP.  
(Note: the UR3e teaching panel can be connected with a key board. You can stop GUI and use terminal mode. Username: root, password: easybot. This seems not necessary but FYI.)
