#!/usr/bin/env python

import yaml
import copy
import rospy
import rospkg
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import Pose, Point
from std_msgs.msg import Int32, Bool, Char
import math
import random

from robotiq_3f_gripper_articulated_msgs.msg import Robotiq3FGripperRobotOutput

FILE_NAME = 'sphere'
FILE_NAMES = ['4_path', '5_path', '7_path']
FILE_NUMS = [100, 100, 100, 100, 100]


D2R = math.pi/180
R2D = 180/math.pi
command = Robotiq3FGripperRobotOutput();
char_command = 'a'

class TrajectoryLoader:
    def __init__(self, file_path, arm_num = 1):
        self.file_path_init = file_path
        self.file_path = file_path
        self.arm_num = arm_num # right arm by default

        # publisher
        self.drawing_line_pub = rospy.Publisher('/ready_to_draw', Bool, queue_size=1)
        self.drawing_color_pub = rospy.Publisher('/drawing_color', Point, queue_size=1)
        self.arm_num_pub = rospy.Publisher('/arm_number', Int32, queue_size=1)

        # gripper
        self.left_gripper_pub = rospy.Publisher('/keft_gripper/Robotiq3FGripperRobotOutput',
                                                 Robotiq3FGripperRobotOutput, queue_size=10)
        self.right_gripper_pub = rospy.Publisher('/right_gripper/Robotiq3FGripperRobotOutput', Robotiq3FGripperRobotOutput, queue_size=10)

        # moveit
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.arms = moveit_commander.MoveGroupCommander("arms")
        self.right_arm = moveit_commander.MoveGroupCommander("right_arm")
        self.right_gripper = moveit_commander.MoveGroupCommander("right_gripper")
        self.left_arm = moveit_commander.MoveGroupCommander("left_arm")
        self.left_gripper = moveit_commander.MoveGroupCommander("left_arm")
        self.right_arm.set_end_effector_link("right_gripper_tool0")
        self.left_arm.set_end_effector_link("left_pen_tool0")


        if self.arm_num == 0:
            self.arm = self.right_arm
            self.gripper = self.right_gripper
        elif self.arm_num == 1:
            self.arm = self.left_arm
            self.gripper = self.left_gripper

        rospy.sleep(3)

    def init_pose (self):
        init_r = [-206 * D2R, -129 * D2R, -50 * D2R, -67 * D2R, 50 * D2R, 98 * D2R]
        init_l = [41 * D2R, -130 * D2R, -101 * D2R, -6 * D2R, 122 * D2R, 6 * D2R]

        joint_goal = self.arms.get_current_joint_values()
        for i in range(6):
            joint_goal[i] = init_l[i]
            joint_goal[6+i] = init_r[i]
        self.arms.go(joint_goal, wait=True)

    def drawing_pose (self):
        self.init_pose()
        sphere_r_pose = self.right_arm.get_current_pose().pose
        sphere_l_pose = self.left_arm.get_current_pose().pose

        # right arm
        linear_path = []
        self.right_arm.set_end_effector_link("right_gripper_tool0")
        sphere_r_pose.position.x = -0.6-0.0078
        sphere_r_pose.position.y = 0.0+0.003
        sphere_r_pose.position.z = 0.95+0.012
        sphere_r_pose.orientation.x = 0.0
        sphere_r_pose.orientation.y = 0.0
        sphere_r_pose.orientation.z = 0.7071068
        sphere_r_pose.orientation.w = -0.7071068

        linear_path.append(copy.deepcopy(sphere_r_pose))
        (plan, fraction) = self.right_arm.compute_cartesian_path(linear_path, 0.01, 0.0)
        if fraction == 1.0 :self.right_arm.execute(plan, wait=True)

        # left arm
        init_pose_traj_path = self.file_path_init + 'init_pose.yaml'

        with open(init_pose_traj_path, 'r') as file_open:
            print('loading ...')
            loaded_plan = yaml.load(file_open)
            # # move to first pose
            # joint_goal = self.left_arm.get_current_joint_values()
            # for j, q in enumerate(loaded_plan.joint_trajectory.points[0].positions):
            #     joint_goal[j] = loaded_plan.joint_trajectory.points[0].positions[j]
            # self.left_arm.go(joint_goal, wait=True)
            self.left_arm.execute(loaded_plan, wait=True)

    def genCommand(self, char, command):
        """Update the command according to the character entered by the user."""

        if char == 'a':
            command = Robotiq3FGripperRobotOutput();
            command.rACT = 1
            command.rGTO = 1
            command.rSPA = 255
            command.rFRA = 150

        if char == 'r':
            command = Robotiq3FGripperRobotOutput();
            command.rACT = 0

        if char == 'c':
            command.rPRA = 255

        if char == 'o':
            command.rPRA = 0

        if char == 'h':
            command.rPRA = 60

        if char == 'b':
            command.rMOD = 0

        if char == 'p':
            command.rMOD = 1

        if char == 'w':
            command.rMOD = 2

        if char == 's':
            command.rMOD = 3

        # If the command entered is a int, assign this value to rPRA
        try:
            command.rPRA = int(char)
            if command.rPRA > 255:
                command.rPRA = 255
            if command.rPRA < 0:
                command.rPRA = 0
        except ValueError:
            pass

        if char == 'f':
            command.rSPA += 25
            if command.rSPA > 255:
                command.rSPA = 255

        if char == 'l':
            command.rSPA -= 25
            if command.rSPA < 0:
                command.rSPA = 0

        if char == 'i':
            command.rFRA += 25
            if command.rFRA > 255:
                command.rFRA = 255

        if char == 'd':
            command.rFRA -= 25
            if command.rFRA < 0:
                command.rFRA = 0

        return command

    def gripper_callback(self, data):
        global char_command, command

        char_command = data
        print(char_command)

        command = self.genCommand(char_command, command)
        self.right_gripper_pub.publish(command)
        rospy.sleep(0.1)

    def sphere_pick (self):
        sphere_pick_traj_path = self.file_path_init + 'sphere_pick.yaml'

        with open(sphere_pick_traj_path, 'r') as file_open:
            print('loading ...')
            loaded_plan = yaml.load(file_open)
            # move to first pose
            joint_goal = self.right_arm.get_current_joint_values()
            for j, q in enumerate(loaded_plan.joint_trajectory.points[-1].positions):
                joint_goal[j] = loaded_plan.joint_trajectory.points[-1].positions[j]
            self.right_arm.go(joint_goal, wait=True)
            for j, q in enumerate(loaded_plan.joint_trajectory.points[0].positions):
                joint_goal[j] = loaded_plan.joint_trajectory.points[0].positions[j]
            self.right_arm.go(joint_goal, wait=True)
            self.gripper_callback('c')
            rospy.sleep(1.5)
            self.right_arm.execute(loaded_plan, wait=True)

    def sphere_place (self):
        sphere_pick_traj_path = self.file_path_init + 'sphere_place.yaml'

        with open(sphere_pick_traj_path, 'r') as file_open:
            print('loading ...')
            loaded_plan = yaml.load(file_open)
            # move to first pose
            joint_goal = self.right_arm.get_current_joint_values()
            for j, q in enumerate(loaded_plan.joint_trajectory.points[0].positions):
                joint_goal[j] = loaded_plan.joint_trajectory.points[0].positions[j]
            self.right_arm.go(joint_goal, wait=True)
            self.right_arm.execute(loaded_plan, wait=True)
            self.gripper_callback('o')
            rospy.sleep(1.5)
            for j, q in enumerate(loaded_plan.joint_trajectory.points[0].positions):
                joint_goal[j] = loaded_plan.joint_trajectory.points[0].positions[j]
            self.right_arm.go(joint_goal, wait=True)
            rospy.sleep(1.0)

    def load_traj(self):
        i = random.randrange(0, len(FILE_NAMES)-1)
        name = FILE_NAMES[i]

        # for num in range(5):
        for num in random.sample(range(100), 15):
            ready = Bool()
            self.file_path = self.file_path_init + FILE_NAME + '_' + name + '_' + str(num) + '.yaml'

            print('file path: ' + self.file_path)
            with open(self.file_path, 'r') as file_open:
                print('loading ...')
                loaded_plan = yaml.load(file_open)
                # move to first pose
                joint_goal = self.left_arm.get_current_joint_values()
                for j, q in enumerate(loaded_plan.joint_trajectory.points[0].positions):
                    joint_goal[j] = loaded_plan.joint_trajectory.points[0].positions[j]
                self.left_arm.go(joint_goal, wait=True)
                self.left_arm.execute(loaded_plan, wait=True)

if __name__ == '__main__':

    rospy.init_node('trajectory_loader', anonymous=True)

    rospack = rospkg.RosPack()
    package_path = rospack.get_path('drawing')
    file_path = package_path + '/data/trajectory/sphere/'  # + FILE_NAME

    tl = TrajectoryLoader(file_path)
    tl.init_pose()

    tl.gripper_callback('a')
    tl.gripper_callback('p')
    rospy.sleep(0.5)


    for iter in range(100):
        tl.sphere_pick()
        tl.drawing_pose()
        tl.gripper_callback('i')
        tl.gripper_callback('i')
        tl.gripper_callback('i')
        rospy.sleep(0.5)
        tl.load_traj()
        tl.init_pose()
        tl.sphere_place()


