#!/usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

class mh:
    def __init__(self):
        pass
        # STANDARD INIT
        rospy.init_node('mh_gtg', anonymous=True)        
        # MOVEIT! INIT
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group_name = "mh_arm"
        self.group = moveit_commander.MoveGroupCommander(self.group_name)
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                            moveit_msgs.msg.DisplayTrajectory,
                                                            queue_size=20)
        # DEFINE PARAMETERS
        self.group.set_goal_position_tolerance(0.1)          # GOAL TOLERANCE
        self.group.set_planning_time(5)                      # TIME TO PLANNING
        self.group.set_max_velocity_scaling_factor(0.8)      # SPEED FACTOR
        # AUX VARIABLES
        self.gtg = geometry_msgs.msg.Pose()
        # INIT VARIABLES
        self.gtg.orientation.x = 0
        self.gtg.orientation.y = 0
        self.gtg.orientation.z = 0
        self.gtg.orientation.w = 1
    # FUNCTION - GET MANUAL COORDINATES TO MOVE MANIPULATOR-H
    def get_coordinates(self):
        print("Usable Workspace Limits X := [-0.6;0.6] | Y := [-0.6;0.6] | Z := [0.16;0.75]")
        self.gtg.position.x = float(input("Enter your X-coordinate: "))
        self.gtg.position.y = float(input("Enter your Y-coordinate: "))
        self.gtg.position.z = float(input("Enter your Z-coordinate: "))
    # FUNCTION - PLANNING TRAJECTORY WITH MORE CONTROL - Verify if the trajectory is OK before running
    def planning_trajectory_controlled(self):
        # 3 - CALCULATE A TRAJECTORY
        self.plan2goal = self.group.plan()
        if self.plan2goal.joint_trajectory.points: # IF TRUE THERE ARE POINTS FOR THE JOINTS => WE HAVE A TRAJECTORY!
            # 4.1 - EXECUTE!
            self.group.execute(self.plan2goal)
            # 4.2 - PREVENT RESIDUAL MOVEMENT
            self.group.stop()
            # 4.3 - CLEAR TARGET GOAL
            self.group.clear_pose_targets()
        else:
            rospy.logerr("Trajectory is empty. Planning was unsuccessful!")
    # FUNCTION - PLANNING TRAJECTORY WITH LESS CONTROL (USE .go COMMAND TO PLAN AND EXECUTE)
    def planning_trajectory_automatic(self):
        # 1 - PLAN AND EXECUTE
        self.group.go(wait=True)
        # 2 - PREVENT RESIDUAL MOVEMENT
        self.group.stop()
        # 3 - CLEAR TARGET GOAL
        self.group.clear_pose_targets()
    # FUNCTION CONTROL MANIPULATOR-H - DEFINED POSITION 
    def go_to_pose(self, position_name):
        ## WE CAN PLAN AND EXECUTE A MOTION FOR THIS GROUP TO A DESIRED SAVED POSE FOR THE END-EFF
        # 1 - PASS YOUR POSE SAVED ON SETUP ASSISTANT
        self.group.set_named_target(position_name)
        # 2 - VERIFY THE PLAN AND EXECUTE
        self.planning_trajectory_controlled() 
    # FUNCTION CONTROL MANIPULATOR-H - MANUAL COORDINATES XYZ
    def go_to_goal(self):
        ## WE CAN PLAN AND EXECUTE A MOTION FOR THIS GROUP TO A DESIRED POSE DEFINED BY USER TO END-EFF
        # 1 - PASS YOUR GOAL
        self.group.set_pose_target(self.gtg)
        # 2 - VERIFY THE PLAN AND EXECUTE
        self.planning_trajectory_automatic() 
    # FUNCTION CONTROL MANIPULATOR-H - JOINT ANGLES
    def set_joints_manually(self, j0, j1, j2, j3, j4, j5):
        # 1 - DEFINE A VARIABLE FORMAT EQUIVALENT TO JOINTS
        joint_goal = self.group.get_current_joint_values()
        # 2 - DEFINE JOINT VALUES
        joint_goal[0] = j0
        joint_goal[1] = j1
        joint_goal[2] = j2
        joint_goal[3] = j3
        joint_goal[4] = j4
        joint_goal[5] = j5
        # 3 - GO!
        self.group.go(joints=joint_goal, wait=True)
        # 4 - STOP CONDITIONS
        self.group.stop()

if __name__ == '__main__':
    try:
        # INITIALIZE YOUR OBJECT
        manip = mh()
        # CONTROL MANIPULATOR-H BY - JOINT ANGLES
        raw_input("Press Enter to control the manipulator by: SETTING JOINTS ANGLES command")
        manip.set_joints_manually(j0 = pi/2, j1 = -pi/2, j2 = -pi/2, j3 = 0, j4 = 0, j5 = pi/2)
        # CONTROL MANIPULATOR-H BY - DEFINED POSITION IN MOVEIT!
        raw_input("Press Enter to control the manipulator by: GO TO A PREDETERMINED POSITION command")
        manip.go_to_pose(position_name="pB")
        # GET YOUR COORDINATES FROM THE USER
        raw_input("Press Enter to control the manipulator by: CHOOSE A RANDOM POINT IN THE WORKSPACE command")
        manip.get_coordinates()
        # CONTROL YOUR MANIPULATOR TO YOUR GOAL
        manip.go_to_goal()

    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        pass