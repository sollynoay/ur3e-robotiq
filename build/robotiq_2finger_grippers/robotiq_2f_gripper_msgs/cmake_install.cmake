# Install script for directory: /home/robot/ur3e-robotiq/src/robotiq_2finger_grippers/robotiq_2f_gripper_msgs

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/robot/ur3e-robotiq/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/robotiq_2f_gripper_msgs/msg" TYPE FILE FILES
    "/home/robot/ur3e-robotiq/src/robotiq_2finger_grippers/robotiq_2f_gripper_msgs/msg/RobotiqGripperCommand.msg"
    "/home/robot/ur3e-robotiq/src/robotiq_2finger_grippers/robotiq_2f_gripper_msgs/msg/RobotiqGripperStatus.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/robotiq_2f_gripper_msgs/action" TYPE FILE FILES "/home/robot/ur3e-robotiq/src/robotiq_2finger_grippers/robotiq_2f_gripper_msgs/action/CommandRobotiqGripper.action")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/robotiq_2f_gripper_msgs/msg" TYPE FILE FILES
    "/home/robot/ur3e-robotiq/devel/share/robotiq_2f_gripper_msgs/msg/CommandRobotiqGripperAction.msg"
    "/home/robot/ur3e-robotiq/devel/share/robotiq_2f_gripper_msgs/msg/CommandRobotiqGripperActionGoal.msg"
    "/home/robot/ur3e-robotiq/devel/share/robotiq_2f_gripper_msgs/msg/CommandRobotiqGripperActionResult.msg"
    "/home/robot/ur3e-robotiq/devel/share/robotiq_2f_gripper_msgs/msg/CommandRobotiqGripperActionFeedback.msg"
    "/home/robot/ur3e-robotiq/devel/share/robotiq_2f_gripper_msgs/msg/CommandRobotiqGripperGoal.msg"
    "/home/robot/ur3e-robotiq/devel/share/robotiq_2f_gripper_msgs/msg/CommandRobotiqGripperResult.msg"
    "/home/robot/ur3e-robotiq/devel/share/robotiq_2f_gripper_msgs/msg/CommandRobotiqGripperFeedback.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/robotiq_2f_gripper_msgs/cmake" TYPE FILE FILES "/home/robot/ur3e-robotiq/build/robotiq_2finger_grippers/robotiq_2f_gripper_msgs/catkin_generated/installspace/robotiq_2f_gripper_msgs-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/robot/ur3e-robotiq/devel/include/robotiq_2f_gripper_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/robot/ur3e-robotiq/devel/share/roseus/ros/robotiq_2f_gripper_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/robot/ur3e-robotiq/devel/share/common-lisp/ros/robotiq_2f_gripper_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/robot/ur3e-robotiq/devel/share/gennodejs/ros/robotiq_2f_gripper_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python3" -m compileall "/home/robot/ur3e-robotiq/devel/lib/python3/dist-packages/robotiq_2f_gripper_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages" TYPE DIRECTORY FILES "/home/robot/ur3e-robotiq/devel/lib/python3/dist-packages/robotiq_2f_gripper_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/robot/ur3e-robotiq/build/robotiq_2finger_grippers/robotiq_2f_gripper_msgs/catkin_generated/installspace/robotiq_2f_gripper_msgs.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/robotiq_2f_gripper_msgs/cmake" TYPE FILE FILES "/home/robot/ur3e-robotiq/build/robotiq_2finger_grippers/robotiq_2f_gripper_msgs/catkin_generated/installspace/robotiq_2f_gripper_msgs-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/robotiq_2f_gripper_msgs/cmake" TYPE FILE FILES
    "/home/robot/ur3e-robotiq/build/robotiq_2finger_grippers/robotiq_2f_gripper_msgs/catkin_generated/installspace/robotiq_2f_gripper_msgsConfig.cmake"
    "/home/robot/ur3e-robotiq/build/robotiq_2finger_grippers/robotiq_2f_gripper_msgs/catkin_generated/installspace/robotiq_2f_gripper_msgsConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/robotiq_2f_gripper_msgs" TYPE FILE FILES "/home/robot/ur3e-robotiq/src/robotiq_2finger_grippers/robotiq_2f_gripper_msgs/package.xml")
endif()

