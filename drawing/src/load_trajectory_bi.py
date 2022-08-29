#!/usr/bin/env python

import yaml
import rospy
import rospkg
import moveit_commander
from geometry_msgs.msg import Pose, Point
from std_msgs.msg import Int32, Bool
import math

FILE_NAME = 'heart_path_bimanual_'
FILE_NUM = 625

D2R = math.pi/180
R2D = 180/math.pi

class TrajectoryLoader:
    def __init__(self, file_path, arm_num = 0):
        self.file_path_init = file_path
        self.file_path = file_path
        self.arm_num = arm_num # right arm by default
        self.data = []

        # publisher
        self.ready_r = rospy.Publisher('/ready_to_draw', Bool, queue_size=1)
        self.ready_l = rospy.Publisher('/ready_to_draw_2', Bool, queue_size=1)
        self.drawing_color_pub = rospy.Publisher('/drawing_color', Point, queue_size=1)
        self.arm_num_pub = rospy.Publisher('/arm_number', Int32, queue_size=1)

        # moveit
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.arms = moveit_commander.MoveGroupCommander("arms")
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
        elif self.arm_num == -1:
            self.arm = self.arms

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

    def load_txt (self, txt_path):
        file = open(txt_path)
        lines = file.readlines
        for line in lines:
            self.data.append(line.split())

    def load_traj (self):
        # self.arm_num_pub.publish(self.arm_num)
        # for i, color in enumerate(FILE_COLORS):
        #     self.set_color(color)
        #     rospy.sleep(1)

        # # point_i_r = 0
        # # point_i_l = 0
        # stroke_i_r = 0
        # stroke_i_l = 0
        # color_i_r = 0
        # color_i_l = 0
        # # point_i_r_prev = -1
        # # point_i_l_prev = -1
        # stroke_i_r_prev = -1
        # stroke_i_l_prev = -1
        # color_i_r_prev = -1
        # color_i_l_prev = -1

        for num in range(FILE_NUM):
            self.file_path = self.file_path_init + str(num) + '.yaml'
            # data = self.data[num]

            # if data[1] == '0':
            #     self.arm = self.right_arm
                # color_i_r = int(data[2])
                # stroke_i_r = int(data[3])
                # point_i_r = int(data[4])
            # elif data[1] == '1':
            #     self.arm = self.left_arm
                # color_i_l = int(data[2])
                # stroke_i_l = int(data[3])
                # point_i_l = int(data[4])
            # elif data[1] == '-1':
            #     self.arm = self.arms
                # color_i_r = int(data[2])
                # stroke_i_r = int(data[3])
                # point_i_r = int(data[4])
                # color_i_l = int(data[5])
                # stroke_i_l = int(data[6])
                # point_i_l = int(data[7])

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
                rospy.sleep(1)
                self.arm.execute(loaded_plan, wait=True)
                # self.ready_r.publish(False)


if __name__ == '__main__':

    rospy.init_node('trajectory_loader', anonymous=True)

    rospack = rospkg.RosPack()
    package_path = rospack.get_path('drawing')
    file_path = package_path + '/data/trajectory/bimanual/' + FILE_NAME
    txt_path = package_path + '/data/trajectory/bimanual/' + FILE_NAME + 'bimanual.txt'

    arm = TrajectoryLoader(file_path, -1)
    # arm.load_txt(txt_path)
    arm.load_traj()

