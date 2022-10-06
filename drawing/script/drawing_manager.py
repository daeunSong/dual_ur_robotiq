#!/usr/bin/env python

import yaml
import rospy
import rospkg
from moveit_msgs.msg import RobotTrajectory
from geometry_msgs.msg import Pose, Point
from std_msgs.msg import Int32, Bool, Char
import moveit_commander
import copy
import math

D2R = math.pi / 180

class DrawingManager:
    def __init__(self, file_path):
        self.file_path_init = file_path
        self.file_path = file_path

        # state publisher
        self.right_command_pub = rospy.Publisher('/right_arm/drawing_command', Int32, queue_size=1)
        self.left_command_pub = rospy.Publisher('/left_arm/drawing_command', Int32, queue_size=1)
        # state subscriber
        self.right_state_sub = rospy.Subscriber('/right_arm/drawing_state', Int32, self.right_state_callback)
        self.left_state_sub = rospy.Subscriber('/left_arm/drawing_state', Int32, self.left_state_callback)

        self.right_state = None
        self.left_state = None

        # moveit
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.right_arm = moveit_commander.MoveGroupCommander("right_arm")
        self.left_arm = moveit_commander.MoveGroupCommander("left_arm")
        self.arms = moveit_commander.MoveGroupCommander("arms")
        self.arm = self.arms
        self.right_arm.set_end_effector_link("right_gripper_tool0")
        self.left_arm.set_end_effector_link("left_gripper_tool0")

        self.init_pose_r = None
        self.init_pose_l = None

        rospy.sleep(3)
        result = False
        # init pose
        while not result:
            # result = self.init_pose_joint()
            # result = self.init_pose_cart()
            result = self.init_pose_lin()

    def right_state_callback (self, msg):
        self.right_state = msg.data
        if self.right_state == 0:
            msg.data = 1
            self.left_command_pub.publish(msg)
        elif self.right_state == 1:
            msg.data = 0
            self.left_command_pub.publish(msg)

    def left_state_callback (self, msg):
        self.left_state = msg.data
        if self.left_state == 0:
            msg.data = 1
            self.right_command_pub.publish(msg)
        elif self.left_state == 1:
            msg.data = 0
            self.right_command_pub.publish(msg)


    def set_pose (self, pose_):
        pose = self.right_arm.get_current_pose().pose
        pose.position.x = pose_[0]
        pose.position.y = pose_[1]
        pose.position.z = pose_[2]
        pose.orientation.x = pose_[3]
        pose.orientation.y = pose_[4]
        pose.orientation.z = pose_[5]
        pose.orientation.w = pose_[6]
        return pose

    def set_pose_joint (self, q):
        joint_goal = self.arm.get_current_joint_values()
        for i in range(len(joint_goal)):
            joint_goal[i] = q[i]

        result = self.arm.go(joint_goal, wait=True)
        rospy.loginfo("Moved to joint pose with state: {}".format(result))
        return result

    def set_pose_cart (self, p):
        self.arm.set_pose_target(self.set_pose(p), self.arm.get_end_effector_link())
        result = self.arms.go()
        rospy.loginfo("Moved to joint pose with state: {}".format(result))
        return result

    def set_pose_lin (self, p):
        waypoints = []
        waypoints.append(copy.deepcopy(self.set_pose(p)))
        (plan, fraction) = self.arm.compute_cartesian_path(
            waypoints,  # waypoints to follow
            0.01,  # eef_step
            0.0)  # jump_threshold
        result = self.arm.execute(plan, wait=True)
        return result

    def init_pose_joint (self):
        self.arm = self.arms
        init_r = [-170*D2R, -120*D2R, -80*D2R, -70*D2R, 45*D2R, 140*D2R]
        init_l = [170*D2R, -65*D2R, 80*D2R, -110*D2R, -45*D2R, 130*D2R]
        q = init_l + init_r
        return self.set_pose_joint(q)

    def init_pose_cart (self):
        pose_r = [-0.65, 0.35, 1.50, -0.7071068, 0.7071068, 0.0, 0.0]
        pose_l = [-0.65, -0.35, 1.50, 0.7071068, 0.7071068, 0.0, 0.0]

        self.arms.set_pose_target(self.set_pose(pose_r), self.right_arm.get_end_effector_link())
        self.arms.set_pose_target(self.set_pose(pose_l), self.left_arm.get_end_effector_link())
        result = self.arms.go()

        rospy.loginfo("Moved to init pose with state: {}".format(result))
        return result

    def init_pose_lin (self, mode='0'):
        pose_r = [-0.65, 0.35, 1.50, -0.7071068, 0.7071068, 0.0, 0.0]
        pose_l = [-0.65, -0.35, 1.50, 0.7071068, 0.7071068, 0.0, 0.0]
        result = False
        if mode == 'r':
            self.arm = self.right_arm
            result = self.set_pose_lin(pose_r)
        elif mode == 'l':
            self.arm = self.left_arm
            result = self.set_pose_lin(pose_l)
        else:
            self.arm = self.right_arm
            self.set_pose_lin(pose_r)
            self.arm = self.left_arm
            result = self.set_pose_lin(pose_l)
        return result




if __name__ == '__main__':
    rospy.init_node('drawing_manager', anonymous=True)

    dm = DrawingManager('')
    # dm.arm = dm.right_arm
    # dm.set_pose_lin([-0.55, 0.10, 1.35, -0.7071068, 0.7071068, 0.0, 0.0])
    # dm.set_pose_lin([-0.65, 0.35, 1.50, -0.7071068, 0.7071068, 0.0, 0.0])
    # dm.arm = dm.left_arm
    # dm.set_pose_lin([-0.65, -0.25, 1.32, 0.7071068, 0.7071068, 0.0, 0.0])
    # dm.set_pose_lin([-0.55, 0.10, 1.35, 0.7071068, 0.7071068, 0.0, 0.0])
    # dm.set_pose_lin([-0.55, -0.00, 1.35, 0.7071068, 0.7071068, 0.0, 0.0])
    # dm.set_pose_lin([-0.55, 0-.25, 1.35, 0.7071068, 0.7071068, 0.0, 0.0])
    # dm.set_pose_lin([-0.65, -0.35, 1.50, 0.7071068, 0.7071068, 0.0, 0.0])

    rospy.spin()


