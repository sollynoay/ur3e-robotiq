# Python 2/3 compatibility imports
from __future__ import print_function
from six.moves import input

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

import numpy as np

import tf.transformations as tr

import roslib


# Brings in the SimpleActionClient
import actionlib

from robotiq_2f_gripper_msgs.msg import CommandRobotiqGripperFeedback, CommandRobotiqGripperResult, CommandRobotiqGripperAction, CommandRobotiqGripperGoal
from robotiq_2f_gripper_control.robotiq_2f_gripper_driver import Robotiq2FingerGripperDriver as Robotiq

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))


from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list




def open_gripper():

    action_name = rospy.get_param('~action_name', 'command_robotiq_action')
    robotiq_client = actionlib.SimpleActionClient(action_name, CommandRobotiqGripperAction)
    # Wait until grippers are ready to take command
    robotiq_client.wait_for_server()

    rospy.logwarn("Client test: Starting sending goals")
    ## Manually set all the parameters of the gripper goal state.
    ######################################################################################

    #goal = CommandRobotiqGripperGoal()
    #goal.emergency_release = False
    ##goal.stop = False
    #goal.position = 0.00
    #goal.speed = 0.1
    #goal.force = 5.0

    # Sends the goal to the gripper.
    #robotiq_client.send_goal(goal)
    # Block processing thread until gripper movement is finished, comment if waiting is not necesary.
    #robotiq_client.wait_for_result()

    # Use pre-defined functions for robot gripper manipulation.
    #####################################################################################
    #while not rospy.is_shutdown():
    Robotiq.goto(robotiq_client, pos=0.06, speed=0.0, force=10 , block=True)
        # Robotiq.goto(robotiq_client, pos=0.06, speed=0.0, force=0)
        # break

    # Prints out the result of executing the action
    return robotiq_client.get_result()  # A FibonacciResult

def close_gripper():

    action_name = rospy.get_param('~action_name', 'command_robotiq_action')
    robotiq_client = actionlib.SimpleActionClient(action_name, CommandRobotiqGripperAction)
    # Wait until grippers are ready to take command
    robotiq_client.wait_for_server()

    rospy.logwarn("Client test: Starting sending goals")
    ## Manually set all the parameters of the gripper goal state.
    ######################################################################################

    goal = CommandRobotiqGripperGoal()
    goal.emergency_release = False
    goal.stop = False
    goal.position = 0.00
    goal.speed = 0.1
    goal.force = 5.0

    # Sends the goal to the gripper.
    robotiq_client.send_goal(goal)
    # Block processing thread until gripper movement is finished, comment if waiting is not necesary.
    robotiq_client.wait_for_result()

    # Use pre-defined functions for robot gripper manipulation.
    #####################################################################################
    #while not rospy.is_shutdown():
    Robotiq.goto(robotiq_client, pos=0.0, speed=0.0, force=1, block=True)
        # Robotiq.goto(robotiq_client, pos=0.06, speed=0.0, force=0)
        # break

    # Prints out the result of executing the action
    return robotiq_client.get_result()  # A FibonacciResult

def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if the values in two lists are within a tolerance of each other.
    For Pose and PoseStamped inputs, the angle between the two quaternions is compared (the angle
    between the identical orientations q and -q is calculated correctly).
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
        # Euclidean distance
        d = dist((x1, y1, z1), (x0, y0, z0))
        # phi = angle between orientations
        cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
        return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

    return True


class MoveGroupPythonInterfaceTutorial(object):
    """MoveGroupPythonInterfaceTutorial"""

    def __init__(self):
        super(MoveGroupPythonInterfaceTutorial, self).__init__()

        ## BEGIN_SUB_TUTORIAL setup
        ##
        ## First initialize `moveit_commander`_ and a `rospy`_ node:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("move_group_python_interface_tutorial", anonymous=True)

        ## kinematic model and the object. Provides information such as the robot's
        ## Instantiate a `RobotCommander`_ oe robot's current joint states
        robot = moveit_commander.RobotCommander()

        ## Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
        ## for getting, setting, and updating the robot's internal understanding of the
        ## surrounding world:
        scene = moveit_commander.PlanningSceneInterface()

        ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
        ## to a planning group (group of joints).  In this tutorial the group is the primary
        ## arm joints in the Panda robot, so we set the group's name to "panda_arm".
        ## If you are using a different robot, change this value to the name of your robot
        ## arm planning group.
        ## This interface can be used to plan and execute motions:
        group_name = "ur3e_arm"
        move_group = moveit_commander.MoveGroupCommander(group_name)

        ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
        ## trajectories in Rviz:
        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )

        ## END_SUB_TUTORIAL

        ## BEGIN_SUB_TUTORIAL basic_info
        ##
        ## Getting Basic Information
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^
        # We can get the name of the reference frame for this robot:
        planning_frame = move_group.get_planning_frame()
        print("============ Planning frame: %s" % planning_frame)

        # We can also print the name of the end-effector link for this group:
        eef_link = move_group.get_end_effector_link()
        print("============ End effector link: %s" % eef_link)

        # We can get a list of all the groups in the robot:
        group_names = robot.get_group_names()
        print("============ Available Planning Groups:", robot.get_group_names())

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        print("============ Printing robot state")
        print(robot.get_current_state())
        print("")
        ## END_SUB_TUTORIAL

        # Misc variables
        self.box_name = ""
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names

    def go_to_joint_state_reach(self):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        move_group = self.move_group

        ## BEGIN_SUB_TUTORIAL plan_to_joint_state
        ##
        ## Planning to a Joint Goal
        ## ^^^^^^^^^^^^^^^^^^^^^^^^
        ## The Panda's zero configuration is at a `singularity <https://www.quora.com/Robotics-What-is-meant-by-kinematic-singularity>`_, so the first
        ## thing we want to do is move it to a slightly better configuration.
        ## We use the constant `tau = 2*pi <https://en.wikipedia.org/wiki/Turn_(angle)#Tau_proposals>`_ for convenience:
        # We get the joint values from the group and change some of the values:
        joint_goal = move_group.get_current_joint_values()
        #joint_goal[0] = -57.86/180*pi  # base
        #joint_goal[1] = -200.51/180*pi       # shoulder
        #joint_goal[2] = 75.36/180*pi  # elbow
        #joint_goal[3] = -144.43/180*pi        # wrist 1
        #joint_goal[4] = 87.57/180*pi          # wrist 2
        #joint_goal[5] = 130.11/180*pi #           # # wrist 3

        joint_goal[0] = -265.32/180*pi  # base
        joint_goal[1] = -106.61/180*pi       # shoulder
        joint_goal[2] = -98.46/180*pi  # elbow
        joint_goal[3] = -61.64/180*pi        # wrist 1
        joint_goal[4] = 89.64/180*pi          # wrist 2
        joint_goal[5] = 91.76/180*pi

        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
        move_group.go(joint_goal, wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
        move_group.stop()

        ## END_SUB_TUTORIAL

        # For testing:
        current_joints = move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)
    
    def go_to_joint_state_home(self):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        move_group = self.move_group

        ## BEGIN_SUB_TUTORIAL plan_to_joint_state
        ##
        ## Planning to a Joint Goal
        ## ^^^^^^^^^^^^^^^^^^^^^^^^
        ## The Panda's zero configuration is at a `singularity <https://www.quora.com/Robotics-What-is-meant-by-kinematic-singularity>`_, so the first
        ## thing we want to do is move it to a slightly better configuration.
        ## We use the constant `tau = 2*pi <https://en.wikipedia.org/wiki/Turn_(angle)#Tau_proposals>`_ for convenience:
        # We get the joint values from the group and change some of the values:
        joint_goal = move_group.get_current_joint_values()
        #joint_goal[0] = -60.67/180*pi  # base
        #joint_goal[1] = -136.14/180*pi       # shoulder
        #joint_goal[2] = 18.74/180*pi  # elbow
        #joint_goal[3] = -150.57/180*pi        # wrist 1
        #joint_goal[4] = 83.95/180*pi          # wrist 2
        #joint_goal[5] = 91.89/180*pi #           # # wrist 3

        joint_goal[0] = -265.32/180*pi # base
        joint_goal[1] = -106.61/180*pi       # shoulder
        joint_goal[2] = -98.46/180*pi  # elbow
        joint_goal[3] = -61.64/180*pi        # wrist 1
        joint_goal[4] = 89.64/180*pi          # wrist 2
        joint_goal[5] = 91.76/180*pi

        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
        move_group.go(joint_goal, wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
        move_group.stop()

        ## END_SUB_TUTORIAL

        # For testing:
        current_joints = move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)
    
    def go_to_joint_state_pick_high(self):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        move_group = self.move_group

        ## BEGIN_SUB_TUTORIAL plan_to_joint_state
        ##
        ## Planning to a Joint Goal
        ## ^^^^^^^^^^^^^^^^^^^^^^^^
        ## The Panda's zero configuration is at a `singularity <https://www.quora.com/Robotics-What-is-meant-by-kinematic-singularity>`_, so the first
        ## thing we want to do is move it to a slightly better configuration.
        ## We use the constant `tau = 2*pi <https://en.wikipedia.org/wiki/Turn_(angle)#Tau_proposals>`_ for convenience:
        # We get the joint values from the group and change some of the values:
        joint_goal = move_group.get_current_joint_values()
        #joint_goal[0] = -60.67/180*pi  # base
        #joint_goal[1] = -136.14/180*pi       # shoulder
        #joint_goal[2] = 18.74/180*pi  # elbow
        #joint_goal[3] = -150.57/180*pi        # wrist 1
        #joint_goal[4] = 83.95/180*pi          # wrist 2
        #joint_goal[5] = 91.89/180*pi #           # # wrist 3

        joint_goal[0] = -263.72/180*pi # base
        joint_goal[1] = -83.54/180*pi       # shoulder
        joint_goal[2] = -48.69/180*pi  # elbow
        joint_goal[3] = -124.42/180*pi        # wrist 1
        joint_goal[4] = 96.45/180*pi          # wrist 2
        joint_goal[5] = 91.76/180*pi

        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
        move_group.go(joint_goal, wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
        move_group.stop()

        ## END_SUB_TUTORIAL

        # For testing:
        current_joints = move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)


    def go_to_pose_goal(self):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        move_group = self.move_group

        ## BEGIN_SUB_TUTORIAL plan_to_pose
        ##
        ## Planning to a Pose Goal
        ## ^^^^^^^^^^^^^^^^^^^^^^^
        ## We can plan a motion for this group to a desired pose for the
        ## end-effector:
        '''
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.x = 0.67639176
        pose_goal.orientation.y = -0.73609055
        pose_goal.orientation.z =  -0.00683402
        pose_goal.orientation.w = 0.02486332
        pose_goal.position.x = -0.00731831#0.0749112800635581
        pose_goal.position.y = 0.47268286#0.6522650966771198
        pose_goal.position.z = 0.0439407
        '''
        '''
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.x =0.3884
        pose_goal.orientation.y = -0.91974
        pose_goal.orientation.z =  0.002793
        pose_goal.orientation.w =  0.0561
        pose_goal.position.x = 0.0908 #0.0749112800635581
        pose_goal.position.y = 0.458#0.6522650966771198
        pose_goal.position.z = -0.0336
        '''
        '''
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.x = 0.96288705
        pose_goal.orientation.y = -0.110849
        pose_goal.orientation.z =  0.2456
        pose_goal.orientation.w = 0.01431571
        pose_goal.position.x = -0.1342#0.0749112800635581
        pose_goal.position.y = 0.4897#0.6522650966771198
        pose_goal.position.z =0.0091
        move_group.set_pose_target(pose_goal)
        
        '''
        '''
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.position.x = -0.105518#0.0749112800635581
        pose_goal.position.y = 0.483699#0.6522650966771198
        pose_goal.position.z =0.00666
        pose_goal.orientation.w =0.01343
        pose_goal.orientation.x =0.98147
        pose_goal.orientation.y =-0.0683379
        pose_goal.orientation.z =0.1784665
        move_group.set_pose_target(pose_goal)
        '''
        '''
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.position.x =0.0383021#0.0749112800635581
        pose_goal.position.y = 0.53216#0.6522650966771198
        pose_goal.position.z =-0.0106568
        pose_goal.orientation.w =-0.0303784
        pose_goal.orientation.x =0.976984
        pose_goal.orientation.y = -0.09549
        pose_goal.orientation.z = 0.1
        move_group.set_pose_target(pose_goal)
        '''
        '''
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.position.x =-0.01115#0.0749112800635581
        pose_goal.position.y = 0.602378#0.6522650966771198
        pose_goal.position.z =-0.044908
        pose_goal.orientation.w = 0.0129072
        pose_goal.orientation.x =0.9723
        pose_goal.orientation.y = -0.22789
        pose_goal.orientation.z = -0.0484651
        move_group.set_pose_target(pose_goal)
        '''
        '''
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.position.x =-0.11810
        pose_goal.position.y = 0.44167
        pose_goal.position.z =0.1494
        pose_goal.orientation.w =-0.0215334
        pose_goal.orientation.x =0.6564
        pose_goal.orientation.y =-0.7540936
        pose_goal.orientation.z =-0.00102957
        move_group.set_pose_target(pose_goal)
        '''
        
        traj = np.loadtxt("/home/robot/catkin_wang/traj11.txt")

        traj = np.array(traj)
        
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.position.x = traj[0,0]
        pose_goal.position.y = traj[0,1]
        pose_goal.position.z =  traj[0,2]
        pose_goal.orientation.w = traj[0,3]
        pose_goal.orientation.x = traj[0,4]
        pose_goal.orientation.y = traj[0,5]
        pose_goal.orientation.z = traj[0,6]



        
        '''
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.position.x = traj[10,0]
        pose_goal.position.y = traj[10,1]
        pose_goal.position.z =  traj[10,2]
        pose_goal.orientation.w = traj[10,3]
        pose_goal.orientation.x = traj[10,4]
        pose_goal.orientation.y = traj[10,5]
        pose_goal.orientation.z = traj[10,6]
        '''
        '''
        traj = np.loadtxt("/home/robot/catkin_wang/traj2.txt")

        traj = np.array(traj)
        
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.position.x = traj[25,0]
        pose_goal.position.y = traj[25,1]
        pose_goal.position.z =  traj[25,2]
        pose_goal.orientation.w = traj[25,3]
        pose_goal.orientation.x = traj[25,4]
        pose_goal.orientation.y = traj[25,5]
        pose_goal.orientation.z = traj[25,6]
        '''
        '''
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.position.x = traj[10,0]
        pose_goal.position.y = traj[10,1]
        pose_goal.position.z =  traj[10,2]
        pose_goal.orientation.w = traj[10,3]
        pose_goal.orientation.x = traj[10,4]
        pose_goal.orientation.y = traj[10,5]
        pose_goal.orientation.z = traj[10,6]
        '''
        move_group.set_pose_target(pose_goal)

        wpose = move_group.get_current_pose().pose


        ## Now, we call the planner to compute the plan and execute it.
        # `go()` returns a boolean indicating whether the planning and execution was successful.
        success = move_group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        move_group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets().
        move_group.clear_pose_targets()

        ## END_SUB_TUTORIAL

        # For testing:
        # Note that since this section of code will not be included in the tutorials
        # we use the class variable rather than the copied state variable
        current_pose = self.move_group.get_current_pose().pose
        return all_close(pose_goal, current_pose, 0.01)

    def plan_cartesian_path_goal(self, scale=1):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        move_group = self.move_group

        ## BEGIN_SUB_TUTORIAL plan_cartesian_path
        ##
        ## Cartesian Paths
        ## ^^^^^^^^^^^^^^^
        ## You can plan a Cartesian path directly by specifying a list of waypoints
        ## for the end-effector to go through. If executing  interactively in a
        ## Python shell, set scale = 1.0.
        ##
        waypoints = []

        wpose = move_group.get_current_pose().pose

        print("x:  ",wpose.position.x)
        print("y: ", wpose.position.y) 
        print("z: ", wpose.position.z)

        traj = np.loadtxt("/home/robot/catkin_wang/exp0126.txt")

        traj = np.array(traj)

        length = traj.shape[0]
        initial_state = self.robot.get_current_state()

        move_group.set_start_state(initial_state)

        #21, 22  27" 28 no" 29

        i = 24
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.position.x = traj[i,0]
        pose_goal.position.y =traj[i,1]
        pose_goal.position.z =  traj[i,2]
        pose_goal.orientation.w =  traj[i,3]
        pose_goal.orientation.x = traj[i,4]
        pose_goal.orientation.y = traj[i,5]
        pose_goal.orientation.z = traj[i,6]

        
       # [-4.76928854 -2.97308012 -1.41417585 -0.31520938  1.55643869  0.81914539]
       # [-4.55559475 -2.87506035 -1.44653036 -0.38414532  1.55466078  1.03285674]


        import tf

        if 1:

            # Your rotation matrix (R) and translation vector (t)
            H = np.loadtxt("/home/robot/trans3.txt")
            R = np.array(H[0:3,0:3])
            #R = np.array([[ 0.9916207 , -0.08755784 , 0.09498427],
            #        [ 0.0853243 ,  0.9959782 ,  0.02733466],
            #        [-0.09699563 ,-0.01900114 , 0.99510341]])

            t = np.array(H[0:3,3])

            print("ttt:",t)
        
            #t = np.array([0.06090738, -0.01720458, 0.00509874])  # Replace with your translation vector

            #t = -R.transpose()@t
            #R = R.transpose()

            # Convert pose_goal to a 4x4 matrix
            transformation_matrix = np.identity(4)
            transformation_matrix[:3, :3] = R
            transformation_matrix[:3, 3] = t

            print(transformation_matrix)

            # Convert pose_goal to a 4x4 matrix
            quaternion = [pose_goal.orientation.x, pose_goal.orientation.y, pose_goal.orientation.z, pose_goal.orientation.w]
            rotation_matrix = tf.transformations.quaternion_matrix(quaternion)
            translation_matrix = tf.transformations.translation_matrix([pose_goal.position.x, pose_goal.position.y, pose_goal.position.z])
            pose_matrix = np.dot(translation_matrix, rotation_matrix)

            # Apply the transformation
            transformed_pose_matrix = np.dot(transformation_matrix,pose_matrix)

            # Convert the transformed matrix back to a Pose message
            transformed_pose = geometry_msgs.msg.Pose()
            transformed_pose.position.x = transformed_pose_matrix[0, 3]
            transformed_pose.position.y = transformed_pose_matrix[1, 3]
            transformed_pose.position.z = traj[i,2]+0.005#transformed_pose_matrix[2, 3]+0.005
            qq = tf.transformations.quaternion_from_matrix(transformed_pose_matrix)
            transformed_pose.orientation.x = qq[0]
            transformed_pose.orientation.y= qq[1]
            transformed_pose.orientation.z= qq[2]
            transformed_pose.orientation.w = qq[3]

            pose_goal = transformed_pose

        else:
            quaternion = [pose_goal.orientation.x, pose_goal.orientation.y, pose_goal.orientation.z, pose_goal.orientation.w]


        
        import tf_conversions
        rot = tf_conversions.transformations.quaternion_matrix(quaternion)#R.from_quat(quaternion)
        rot = rot[:3,:3]#.transpose()#@RR.transpose()*s
        rot = np.array([rot[:,2],-rot[:,0],-rot[:,1]]).transpose()

        A1 = np.zeros((4,4))
                                

        A1[:3,:3]=rot
        A1[3,3]=1
        tt = np.array([pose_goal.position.x,pose_goal.position.y,pose_goal.position.z])
        A1[0:3,3]=tt
        q=np.array([-265.32/180*np.pi,-106.61/180*np.pi,-98.46/180*np.pi,
                    -61.64/180*np.pi,89.64/180*np.pi,91.76/180*np.pi])
        

        
        #if qsol[0]>-360 and qsol[0]<360:
        #    move_group.go(joint_goal, wait=True)

        move_group.set_pose_target(pose_goal) #pose_goal

        move_group.set_planner_id("RRTConnect")
        move_group.set_planning_time(5)
        #plan = moveit_msgs.msg.RobotTrajectory()
        #plan = move_group.plan()

        plan = moveit_msgs.msg.RobotTrajectory()
        plan = move_group.plan()
        '''
        for i in range(length):
            pose_goal = geometry_msgs.msg.Pose()
            pose_goal.position.x = traj[i,0]
            pose_goal.position.y = traj[i,1]
            pose_goal.position.z =  traj[i,2]
            pose_goal.orientation.w = traj[i,3]
            pose_goal.orientation.x = traj[i,4]
            pose_goal.orientation.y = traj[i,5]
            pose_goal.orientation.z = traj[i,6]

        
            waypoints.append(copy.deepcopy(pose_goal))

        (plan, fraction) = move_group.compute_cartesian_path(
            waypoints, 0.005, 0.00  # waypoints to follow  # eef_step
        )  # jump_threshold
        '''
        # Note: We are just planning, not asking move_group to actually move the robot yet:
        return plan#, fraction

        ## END_SUB_TUTORIAL
    def plan_cartesian_path(self, scale=1):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        move_group = self.move_group

        ## BEGIN_SUB_TUTORIAL plan_cartesian_path
        ##
        ## Cartesian Paths
        ## ^^^^^^^^^^^^^^^
        ## You can plan a Cartesian path directly by specifying a list of waypoints
        ## for the end-effector to go through. If executing  interactively in a
        ## Python shell, set scale = 1.0.
        ##
        waypoints = []

        wpose = move_group.get_current_pose().pose

        print("x:  ",wpose.position.x)
        print("y: ", wpose.position.y) 
        print("z: ", wpose.position.z)

        traj = np.loadtxt("/home/robot/catkin_wang/demo_traj.txt")

        traj = np.array(traj)

        length = traj.shape[0]
        initial_state = self.robot.get_current_state()

        move_group.set_start_state(initial_state)

        #21, 22  27" 28 no" 29
        waypoints = []
        #i = 45
        for i in range(length):
            pose_goal = geometry_msgs.msg.Pose()
            pose_goal.position.x = traj[i,0]
            pose_goal.position.y =traj[i,1]
            pose_goal.position.z =  traj[i,2]
            pose_goal.orientation.w =  traj[i,3]
            pose_goal.orientation.x = traj[i,4]
            pose_goal.orientation.y = traj[i,5]
            pose_goal.orientation.z = traj[i,6]
            if i==21 or i==22 or i==27 or i==44 or i==45 :
                waypoints.append(copy.deepcopy(pose_goal))
            
        '''
            matrix = tr.quaternion_matrix([pose_goal.orientation.x,pose_goal.orientation.y,pose_goal.orientation.z,pose_goal.orientation.w])

            matrix[0:3,3] = [pose_goal.position.x,pose_goal.position.y,pose_goal.position.z]
            q=np.array([-57.86/180*np.pi,-200.51/180*np.pi,75.36/180*np.pi,
                        -144.43/180*np.pi,87.57/180*np.pi,130.11/180*np.pi])
            sols = inverse(np.array(matrix),q[5])
            for sol in sols:
                print("aaa")
            qsol = best_sol(sols, q, [1.]*6)
            if qsol is None:
                qsol = [999.]*6
    
            print(qsol)
        '''
 


        move_group.set_planner_id("RRTConnect")
        move_group.set_planning_time(5)
        #plan = moveit_msgs.msg.RobotTrajectory()
        #plan = move_group.plan()
        (plan, fraction) = move_group.compute_cartesian_path(
            waypoints, 0.005, 0.00  # waypoints to follow  # eef_step
        )
        '''
        for i in range(length):
            pose_goal = geometry_msgs.msg.Pose()
            pose_goal.position.x = traj[i,0]
            pose_goal.position.y = traj[i,1]
            pose_goal.position.z =  traj[i,2]
            pose_goal.orientation.w = traj[i,3]
            pose_goal.orientation.x = traj[i,4]
            pose_goal.orientation.y = traj[i,5]
            pose_goal.orientation.z = traj[i,6]

        
            waypoints.append(copy.deepcopy(pose_goal))

        (plan, fraction) = move_group.compute_cartesian_path(
            waypoints, 0.005, 0.00  # waypoints to follow  # eef_step
        )  # jump_threshold
        '''
        # Note: We are just planning, not asking move_group to actually move the robot yet:
        return plan#, fraction

        ## END_SUB_TUTORIAL
    def display_trajectory(self, plan):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        robot = self.robot
        display_trajectory_publisher = self.display_trajectory_publisher

        ## BEGIN_SUB_TUTORIAL display_trajectory
        ##
        ## Displaying a Trajectory
        ## ^^^^^^^^^^^^^^^^^^^^^^^
        ## You can ask RViz to visualize a plan (aka trajectory) for you. But the
        ## group.plan() method does this automatically so this is not that useful
        ## here (it just displays the same trajectory again):
        ##
        ## A `DisplayTrajectory`_ msg has two primary fields, trajectory_start and trajectory.
        ## We populate the trajectory_start with our current robot state to copy over
        ## any AttachedCollisionObjects and add our plan to the trajectory.
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        # Publish
        display_trajectory_publisher.publish(display_trajectory[1])

        ## END_SUB_TUTORIAL

    def execute_plan(self, plan):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        move_group = self.move_group

        ## BEGIN_SUB_TUTORIAL execute_plan
        ##
        ## Executing a Plan
        ## ^^^^^^^^^^^^^^^^
        ## Use execute if you would like the robot to follow
        ## the plan that has already been computed:
        move_group.execute(plan, wait=True)

        wpose = move_group.get_current_pose().pose

        print("x:  ",wpose.position.x)
        print("y: ", wpose.position.y) 
        print("z: ", wpose.position.z)

        ## **Note:** The robot's current joint state must be within some tolerance of the
        ## first waypoint in the `RobotTrajectory`_ or ``execute()`` will fail
        ## END_SUB_TUTORIAL

    def wait_for_state_update(
        self, box_is_known=False, box_is_attached=False, timeout=4
    ):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.box_name
        scene = self.scene

        ## BEGIN_SUB_TUTORIAL wait_for_scene_update
        ##
        ## Ensuring Collision Updates Are Received
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## If the Python node was just created (https://github.com/ros/ros_comm/issues/176),
        ## or dies before actually publishing the scene update message, the message
        ## could get lost and the box will not appear. To ensure that the updates are
        ## made, we wait until we see the changes reflected in the
        ## ``get_attached_objects()`` and ``get_known_object_names()`` lists.
        ## For the purpose of this tutorial, we call this function after adding,
        ## removing, attaching or detaching an object in the planning scene. We then wait
        ## until the updates have been made or ``timeout`` seconds have passed.
        ## To avoid waiting for scene updates like this at all, initialize the
        ## planning scene interface with  ``synchronous = True``.
        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            # Test if the box is in attached objects
            attached_objects = scene.get_attached_objects([box_name])
            is_attached = len(attached_objects.keys()) > 0

            # Test if the box is in the scene.
            # Note that attaching the box will remove it from known_objects
            is_known = box_name in scene.get_known_object_names()

            # Test if we are in the expected state
            if (box_is_attached == is_attached) and (box_is_known == is_known):
                return True

            # Sleep so that we give other threads time on the processor
            rospy.sleep(0.1)
            seconds = rospy.get_time()

        # If we exited the while loop without returning then we timed out
        return False
        ## END_SUB_TUTORIAL

    def add_box(self, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.box_name
        scene = self.scene

        ## BEGIN_SUB_TUTORIAL add_box
        ##
        ## Adding Objects to the Planning Scene
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## First, we will create a box in the planning scene between the fingers:
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "panda_hand"
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.z = 0.11  # above the panda_hand frame
        box_name = "box"
        scene.add_box(box_name, box_pose, size=(0.075, 0.075, 0.075))

        ## END_SUB_TUTORIAL
        # Copy local variables back to class variables. In practice, you should use the class
        # variables directly unless you have a good reason not to.
        self.box_name = box_name
        return self.wait_for_state_update(box_is_known=True, timeout=timeout)

    def attach_box(self, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.box_name
        robot = self.robot
        scene = self.scene
        eef_link = self.eef_link
        group_names = self.group_names

        ## BEGIN_SUB_TUTORIAL attach_object
        ##
        ## Attaching Objects to the Robot
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## Next, we will attach the box to the Panda wrist. Manipulating objects requires the
        ## robot be able to touch them without the planning scene reporting the contact as a
        ## collision. By adding link names to the ``touch_links`` array, we are telling the
        ## planning scene to ignore collisions between those links and the box. For the Panda
        ## robot, we set ``grasping_group = 'hand'``. If you are using a different robot,
        ## you should change this value to the name of your end effector group name.
        grasping_group = "panda_hand"
        touch_links = robot.get_link_names(group=grasping_group)
        scene.attach_box(eef_link, box_name, touch_links=touch_links)
        ## END_SUB_TUTORIAL

        # We wait for the planning scene to update.
        return self.wait_for_state_update(
            box_is_attached=True, box_is_known=False, timeout=timeout
        )

    def detach_box(self, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.box_name
        scene = self.scene
        eef_link = self.eef_link

        ## BEGIN_SUB_TUTORIAL detach_object
        ##
        ## Detaching Objects from the Robot
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## We can also detach and remove the object from the planning scene:
        scene.remove_attached_object(eef_link, name=box_name)
        ## END_SUB_TUTORIAL

        # We wait for the planning scene to update.
        return self.wait_for_state_update(
            box_is_known=True, box_is_attached=False, timeout=timeout
        )

    def remove_box(self, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.box_name
        scene = self.scene

        ## BEGIN_SUB_TUTORIAL remove_object
        ##
        ## Removing Objects from the Planning Scene
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## We can remove the box from the world.
        scene.remove_world_object(box_name)

        ## **Note:** The object must be detached before we can remove it from the world
        ## END_SUB_TUTORIAL

        # We wait for the planning scene to update.
        return self.wait_for_state_update(
            box_is_attached=False, box_is_known=False, timeout=timeout
        )


def main():
    try:
        print("")
        print("----------------------------------------------------------")
        print("Welcome to the MoveIt MoveGroup Python Interface Tutorial")
        print("----------------------------------------------------------")
        print("Press Ctrl-D to exit at any time")
        print("")
        input(
            "============ Press `Enter` to begin the tutorial by setting up the moveit_commander ..."
        )
        tutorial = MoveGroupPythonInterfaceTutorial()

        #input(
        #    "============ Press `Enter` to execute a movement using a joint state goal ..."
        #)
        tutorial.go_to_joint_state_home()
        input(
            "============ reach ..."
        )
        open_gripper()
    
        #tutorial.go_to_joint_state_reach()

        #input("============ Press `Enter` to execute a movement using a pose goal ...")
        #tutorial.go_to_pose_goal()

        input("============ Press `Enter` to plan and display a Cartesian path ...")
        #cartesian_plan = tutorial.plan_cartesian_path() #, fraction 

        input("============ Press `Enter` to execute a saved path ...")
        #tutorial.execute_plan(cartesian_plan)

        input("============ Press `Enter` to plan and display a Cartesian path ...")
        #cartesian_plan = tutorial.plan_cartesian_path_goal() #, fraction 

        input("============ Press `Enter` to execute a saved path ...")
        #tutorial.execute_plan(cartesian_plan[1])
        #tutorial.execute_plan(cartesian_plan)
     
        input("============ grasp...")
        close_gripper()
        input(
            "============ reach ..."
        )
        tutorial.go_to_joint_state_home()
        #tutorial.go_to_joint_state()
        tutorial.go_to_joint_state_pick_high()

        #
        #tutorial.add_box()

        #input("============ Press `Enter` to attach a Box to the Panda robot ...")
        #tutorial.attach_box()

        #input(
        #    "============ Press `Enter` to plan and execute a path with an attached collision object ..."
        #)
        #cartesian_plan, fraction = tutorial.plan_cartesian_path(scale=-1)
        #tutorial.execute_plan(cartesian_plan)

        #input("============ Press `Enter` to detach the box from the Panda robot ...")
        #tutorial.detach_box()

        #input(
        #    "============ Press `Enter` to remove the box from the planning scene ..."
        #)
        #tutorial.remove_box()

        print("============ Python tutorial demo complete!")
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return

if __name__ == "__main__":
    main()