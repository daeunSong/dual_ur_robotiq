#!/usr/bin/env python

import yaml
import rospy
import rospkg
import moveit_commander
from geometry_msgs.msg import Pose, Point
from std_msgs.msg import Int32, Bool
import math

FILE_NAME = 'heart_path_'
FILE_COLORS = ['c', 'm', 'y', 'k']
FILE_NUMS = [10, 26, 2, 8]

D2R = math.pi/180
R2D = 180/math.pi

class TrajectoryLoader:
    def __init__(self, file_path, arm_num = 0):
        self.file_path_init = file_path
        self.file_path = file_path
        self.arm_num = arm_num # right arm by default

        # publisher
        self.drawing_line_pub = rospy.Publisher('/ready_to_draw', Bool, queue_size=1)
        self.drawing_color_pub = rospy.Publisher('/drawing_color', Point, queue_size=1)
        self.arm_num_pub = rospy.Publisher('/arm_number', Int32, queue_size=1)

        # moveit
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.right_arm = moveit_commander.MoveGroupCommander("left_arm")
        self.right_gripper = moveit_commander.MoveGroupCommander("left_gripper")
        self.left_arm = moveit_commander.MoveGroupCommander("right_arm")
        self.left_gripper = moveit_commander.MoveGroupCommander("right_arm")
        self.right_arm.set_end_effector_link("left_gripper_tool0")
        self.left_arm.set_end_effector_link("right_gripper_tool0")

        if self.arm_num == 0:
            self.arm = self.right_arm
            self.gripper = self.right_gripper
        elif self.arm_num == 1:
            self.arm = self.left_arm
            self.gripper = self.left_gripper

        rospy.sleep(3)
        self.init_pose()

    def init_pose (self):
        init_r = [172*D2R, -97*D2R, -106*D2R, -73*D2R, -44*D2R, -35*D2R]
        init_l = [-178*D2R, -88*D2R, 97*D2R, -102*D2R, 46*D2R, -43*D2R]

        joint_goal = self.arm.get_current_joint_values()
        for i in range(6):
            if self.arm_num == 0: joint_goal[i] = init_r[i]
            if self.arm_num == 1: joint_goal[i] = init_l[i]
        self.arm.go(joint_goal, wait=True)

    def set_color (self, color):
        c = Point()
        if color == 'c':
            c.x = 0.0; c.y = 1.0; c.z = 1.0
        elif color == 'm':
            c.x = 1.0; c.y = 0.0; c.z = 1.0
        elif color == 'y':
            c.x = 1.0; c.y = 1.0; c.z = 0.0
        else : # 'k'
            c.x = 0.0; c.y = 0.0; c.z = 0.0
        self.drawing_color_pub.publish(c)

    def load_traj (self):
        self.arm_num_pub.publish(self.arm_num)
        for i, color in enumerate(FILE_COLORS):
            self.set_color(color)
            rospy.sleep(1)

            for num in range(FILE_NUMS[i]):
                ready = Bool()
                if self.arm_num == 0: # right
                    self.file_path = self.file_path_init + 'r_' + color + '_' + str(num) + '.yaml'
                elif self.arm_num == 1: #left
                    self.file_path = self.file_path_init + 'l_' + color + '_' + str(num) + '.yaml'

                print('file path: ' + self.file_path)
                with open(self.file_path, 'r') as file_open:
                    print('loading ...')
                    loaded_plan = yaml.load(file_open)
                    # move to first pose
                    joint_goal = self.arm.get_current_joint_values()
                    for j, q in enumerate(loaded_plan.joint_trajectory.points[0].positions):
                        joint_goal[j] = loaded_plan.joint_trajectory.points[0].positions[j]
                    self.arm.go(joint_goal, wait=True)
                    # draw
                    self.drawing_line_pub.publish(True)
                    rospy.sleep(1)
                    print('drawing ' + color + ' ' + str(num) + 'th stroke')
                    self.arm.execute(loaded_plan, wait=True)
                    self.drawing_line_pub.publish(False)


if __name__ == '__main__':

    rospy.init_node('trajectory_loader', anonymous=True)

    rospack = rospkg.RosPack()
    package_path = rospack.get_path('drawing')
    file_path = package_path + '/data/trajectory/' + FILE_NAME

    right = TrajectoryLoader(file_path, 0)
    # left = TrajectoryLoader(file_path, 1)
    right.load_traj()
    # left.load_traj()

