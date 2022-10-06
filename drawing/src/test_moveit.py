#!/usr/bin/env python

import yaml
import rospy
import rospkg
import moveit_commander
from geometry_msgs.msg import Pose, Point
from std_msgs.msg import Int32, Bool
import math
import copy

FILE_NAME = 'drawing_test_'
# FILE_COLORS = ['c', 'm', 'y', 'k']
# FILE_NUMS = [10, 26, 2, 8]
FILE_COLORS = ['k']
FILE_NUMS = [3]


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
        self.right_arm = moveit_commander.MoveGroupCommander("right_arm")
        # self.right_gripper = moveit_commander.MoveGroupCommander("right_gripper")
        self.left_arm = moveit_commander.MoveGroupCommander("left_arm")
        # self.left_gripper = moveit_commander.MoveGroupCommander("left_arm")
        self.right_arm.set_end_effector_link("right_gripper_tool0")
        self.left_arm.set_end_effector_link("left_gripper_tool0")

        self.init_pose_r = self.right_arm.get_current_pose().pose
        self.init_pose_l = self.left_arm.get_current_pose().pose

        if self.arm_num == 0:
            self.arm = self.right_arm
            # self.gripper = self.right_gripper
        elif self.arm_num == 1:
            self.arm = self.left_arm
            # self.gripper = self.left_gripper

        rospy.sleep(3)

    def init_pose_cart (self):
        waypoints = []
        wpose = self.right_arm.get_current_pose().pose
        wpose.position.x = -0.65
        wpose.position.y = 0.32
        wpose.position.z = 1.50
        wpose.orientation.x = -0.7071068
        wpose.orientation.y = 0.7071068
        wpose.orientation.z = 0.0
        wpose.orientation.w = 0.0
        waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = self.right_arm.compute_cartesian_path(
            waypoints,  # waypoints to follow
            0.01,  # eef_step
            0.0)  # jump_threshold

        self.right_arm.execute(plan, wait=True)

    def left_pick_1(self):
        rospack = rospkg.RosPack()
        package_path = rospack.get_path('drawing')

        waypoints = []
        wpose = self.left_arm.get_current_pose().pose
        wpose.position.x = -0.8
        wpose.position.y = -0.273
        wpose.position.z = 1.32
        wpose.orientation.x = 0.0#-0.7071068
        wpose.orientation.y = 1.0#0.7071068
        wpose.orientation.z = 0.0
        wpose.orientation.w = 0.0
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.x = -0.98
        waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = self.left_arm.compute_cartesian_path(
            waypoints,  # waypoints to follow
            0.01,  # eef_step
            0.0)  # jump_threshold

        file_path = package_path + '/data/trajectory/tool/pick_approach_l_1.yaml'
        # dump the plan into yaml file
        with open(file_path, 'w') as file_save:
            yaml.dump(plan, file_save, default_flow_style=True)

        self.left_arm.execute(plan, wait=True)

        raw_input("wait for input h")  #h

        # move up
        waypoints = []
        wpose.position.z = 1.45
        waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = self.left_arm.compute_cartesian_path(
            waypoints,  # waypoints to follow
            0.01,  # eef_step
            0.0)  # jump_threshold

        file_path = package_path + '/data/trajectory/tool/pick_up_l_1.yaml'
        # dump the plan into yaml file
        with open(file_path, 'w') as file_save:
            yaml.dump(plan, file_save, default_flow_style=True)

        self.left_arm.execute(plan, wait=True)

        raw_input("wait for input c")  # c

        waypoints = []
        waypoints.append(copy.deepcopy(self.init_pose_l))

        (plan, fraction) = self.left_arm.compute_cartesian_path(
            waypoints,  # waypoints to follow
            0.01,  # eef_step
            0.0)  # jump_threshold

        file_path = package_path + '/data/trajectory/tool/pick_out_l_1.yaml'
        # dump the plan into yaml file
        with open(file_path, 'w') as file_save:
            yaml.dump(plan, file_save, default_flow_style=True)

        self.left_arm.execute(plan, wait=True)

    def left_place_1 (self):
        rospack = rospkg.RosPack()
        package_path = rospack.get_path('drawing')

        waypoints = []
        wpose = self.left_arm.get_current_pose().pose
        wpose.position.x = -0.98
        wpose.position.y = -0.273
        wpose.position.z = 1.45
        wpose.orientation.x = 0.0  # -0.7071068
        wpose.orientation.y = 1.0  # 0.7071068
        wpose.orientation.z = 0.0
        wpose.orientation.w = 0.0
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.z = 1.40
        waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = self.left_arm.compute_cartesian_path(
            waypoints,  # waypoints to follow
            0.01,  # eef_step
            0.0)  # jump_threshold

        file_path = package_path + '/data/trajectory/tool/place_ready_l_1.yaml'
        # dump the plan into yaml file
        with open(file_path, 'w') as file_save:
            yaml.dump(plan, file_save, default_flow_style=True)

        self.left_arm.execute(plan, wait=True)

        raw_input("wait for input h")  # h

        # place
        waypoints = []
        wpose.position.z = 1.32
        waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = self.left_arm.compute_cartesian_path(
            waypoints,  # waypoints to follow
            0.01,  # eef_step
            0.0)  # jump_threshold

        file_path = package_path + '/data/trajectory/tool/place_down_l_1.yaml'
        # dump the plan into yaml file
        with open(file_path, 'w') as file_save:
            yaml.dump(plan, file_save, default_flow_style=True)

        self.left_arm.execute(plan, wait=True)

        raw_input("wait for input o")  # o

        # back, side, forward
        waypoints = []
        wpose.position.x = -0.8
        waypoints.append(copy.deepcopy(wpose))
        waypoints.append(copy.deepcopy(self.init_pose_l))

        (plan, fraction) = self.left_arm.compute_cartesian_path(
            waypoints,  # waypoints to follow
            0.01,  # eef_step
            0.0)  # jump_threshold

        file_path = package_path + '/data/trajectory/tool/place_out_l_1.yaml'
        # dump the plan into yaml file
        with open(file_path, 'w') as file_save:
            yaml.dump(plan, file_save, default_flow_style=True)

        self.left_arm.execute(plan, wait=True)

    def left_pick_2(self):
        rospack = rospkg.RosPack()
        package_path = rospack.get_path('drawing')

        waypoints = []
        wpose = self.left_arm.get_current_pose().pose
        wpose.position.x = -0.8
        wpose.position.y = -0.473
        wpose.position.z = 1.32
        wpose.orientation.x = 0.0  # -0.7071068
        wpose.orientation.y = 1.0  # 0.7071068
        wpose.orientation.z = 0.0
        wpose.orientation.w = 0.0
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.x = -0.98
        waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = self.left_arm.compute_cartesian_path(
            waypoints,  # waypoints to follow
            0.01,  # eef_step
            0.0)  # jump_threshold

        file_path = package_path + '/data/trajectory/tool/pick_approach_l_2.yaml'
        # dump the plan into yaml file
        with open(file_path, 'w') as file_save:
            yaml.dump(plan, file_save, default_flow_style=True)

        self.left_arm.execute(plan, wait=True)

        raw_input("wait for input h")  # h

        # move up
        waypoints = []
        wpose.position.z = 1.45
        waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = self.left_arm.compute_cartesian_path(
            waypoints,  # waypoints to follow
            0.01,  # eef_step
            0.0)  # jump_threshold

        file_path = package_path + '/data/trajectory/tool/pick_up_l_2.yaml'
        # dump the plan into yaml file
        with open(file_path, 'w') as file_save:
            yaml.dump(plan, file_save, default_flow_style=True)

        self.left_arm.execute(plan, wait=True)

        raw_input("wait for input c")  # c

        waypoints = []
        waypoints.append(copy.deepcopy(self.init_pose_l))

        (plan, fraction) = self.left_arm.compute_cartesian_path(
            waypoints,  # waypoints to follow
            0.01,  # eef_step
            0.0)  # jump_threshold

        file_path = package_path + '/data/trajectory/tool/pick_out_l_2.yaml'
        # dump the plan into yaml file
        with open(file_path, 'w') as file_save:
            yaml.dump(plan, file_save, default_flow_style=True)

        self.left_arm.execute(plan, wait=True)

    def left_place_2(self):
        rospack = rospkg.RosPack()
        package_path = rospack.get_path('drawing')

        waypoints = []
        wpose = self.left_arm.get_current_pose().pose
        wpose.position.x = -0.96
        wpose.position.y = -0.473
        wpose.position.z = 1.45
        wpose.orientation.x = 0.0  # -0.7071068
        wpose.orientation.y = 1.0  # 0.7071068
        wpose.orientation.z = 0.0
        wpose.orientation.w = 0.0
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.z = 1.40
        waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = self.left_arm.compute_cartesian_path(
            waypoints,  # waypoints to follow
            0.01,  # eef_step
            0.0)  # jump_threshold

        file_path = package_path + '/data/trajectory/tool/place_ready_l_2.yaml'
        # dump the plan into yaml file
        with open(file_path, 'w') as file_save:
            yaml.dump(plan, file_save, default_flow_style=True)

        self.left_arm.execute(plan, wait=True)

        raw_input("wait for input h")  # h

        # place
        waypoints = []
        wpose.position.z = 1.32
        waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = self.left_arm.compute_cartesian_path(
            waypoints,  # waypoints to follow
            0.01,  # eef_step
            0.0)  # jump_threshold

        file_path = package_path + '/data/trajectory/tool/place_down_l_2.yaml'
        # dump the plan into yaml file
        with open(file_path, 'w') as file_save:
            yaml.dump(plan, file_save, default_flow_style=True)

        self.left_arm.execute(plan, wait=True)

        raw_input("wait for input o")  # o

        # back, side, forward
        waypoints = []
        wpose.position.x = -0.8
        waypoints.append(copy.deepcopy(wpose))
        waypoints.append(copy.deepcopy(self.init_pose_l))

        (plan, fraction) = self.left_arm.compute_cartesian_path(
            waypoints,  # waypoints to follow
            0.01,  # eef_step
            0.0)  # jump_threshold

        file_path = package_path + '/data/trajectory/tool/place_out_l_2.yaml'
        # dump the plan into yaml file
        with open(file_path, 'w') as file_save:
            yaml.dump(plan, file_save, default_flow_style=True)

        self.left_arm.execute(plan, wait=True)

    def right_pick_1(self):
        rospack = rospkg.RosPack()
        package_path = rospack.get_path('drawing')

        waypoints = []
        wpose = self.right_arm.get_current_pose().pose
        wpose.position.x = -0.8
        wpose.position.y = 0.255
        wpose.position.z = 1.32
        wpose.orientation.x = 0.0#-0.7071068
        wpose.orientation.y = 1.0#0.7071068
        wpose.orientation.z = 0.0
        wpose.orientation.w = 0.0
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.x = -0.96
        waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = self.right_arm.compute_cartesian_path(
            waypoints,  # waypoints to follow
            0.01,  # eef_step
            0.0)  # jump_threshold

        file_path = package_path + '/data/trajectory/tool/pick_approach_r_1.yaml'
        # dump the plan into yaml file
        with open(file_path, 'w') as file_save:
            yaml.dump(plan, file_save, default_flow_style=True)

        self.right_arm.execute(plan, wait=True)

        raw_input("wait for input h")  #h

        # move up
        waypoints = []
        wpose.position.z = 1.45
        waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = self.right_arm.compute_cartesian_path(
            waypoints,  # waypoints to follow
            0.01,  # eef_step
            0.0)  # jump_threshold

        file_path = package_path + '/data/trajectory/tool/pick_up_r_1.yaml'
        # dump the plan into yaml file
        with open(file_path, 'w') as file_save:
            yaml.dump(plan, file_save, default_flow_style=True)

        self.right_arm.execute(plan, wait=True)

        raw_input("wait for input c")  # c

        waypoints = []
        waypoints.append(copy.deepcopy(self.init_pose_r))

        (plan, fraction) = self.right_arm.compute_cartesian_path(
            waypoints,  # waypoints to follow
            0.01,  # eef_step
            0.0)  # jump_threshold

        file_path = package_path + '/data/trajectory/tool/pick_out_r_1.yaml'
        # dump the plan into yaml file
        with open(file_path, 'w') as file_save:
            yaml.dump(plan, file_save, default_flow_style=True)

        self.right_arm.execute(plan, wait=True)

    def right_place_1 (self):
        rospack = rospkg.RosPack()
        package_path = rospack.get_path('drawing')

        waypoints = []
        wpose = self.right_arm.get_current_pose().pose
        wpose.position.x = -0.96
        wpose.position.y = 0.255
        wpose.position.z = 1.45
        wpose.orientation.x = 0.0  # -0.7071068
        wpose.orientation.y = 1.0  # 0.7071068
        wpose.orientation.z = 0.0
        wpose.orientation.w = 0.0
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.z = 1.40
        waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = self.right_arm.compute_cartesian_path(
            waypoints,  # waypoints to follow
            0.01,  # eef_step
            0.0)  # jump_threshold

        file_path = package_path + '/data/trajectory/tool/place_ready_r_1.yaml'
        # dump the plan into yaml file
        with open(file_path, 'w') as file_save:
            yaml.dump(plan, file_save, default_flow_style=True)

        self.right_arm.execute(plan, wait=True)

        raw_input("wait for input h")  # h

        # place
        waypoints = []
        wpose.position.z = 1.32
        waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = self.right_arm.compute_cartesian_path(
            waypoints,  # waypoints to follow
            0.01,  # eef_step
            0.0)  # jump_threshold

        file_path = package_path + '/data/trajectory/tool/place_down_r_1.yaml'
        # dump the plan into yaml file
        with open(file_path, 'w') as file_save:
            yaml.dump(plan, file_save, default_flow_style=True)

        self.right_arm.execute(plan, wait=True)

        raw_input("wait for input o")  # o

        # back, side, forward
        waypoints = []
        wpose.position.x = -0.8
        waypoints.append(copy.deepcopy(wpose))
        waypoints.append(copy.deepcopy(self.init_pose_r))

        (plan, fraction) = self.right_arm.compute_cartesian_path(
            waypoints,  # waypoints to follow
            0.01,  # eef_step
            0.0)  # jump_threshold

        file_path = package_path + '/data/trajectory/tool/place_out_r_1.yaml'
        # dump the plan into yaml file
        with open(file_path, 'w') as file_save:
            yaml.dump(plan, file_save, default_flow_style=True)

        self.right_arm.execute(plan, wait=True)

    def right_pick_2(self):
        rospack = rospkg.RosPack()
        package_path = rospack.get_path('drawing')

        waypoints = []
        wpose = self.right_arm.get_current_pose().pose
        wpose.position.x = -0.8
        wpose.position.y = 0.455
        wpose.position.z = 1.32
        wpose.orientation.x = 0.0  # -0.7071068
        wpose.orientation.y = 1.0  # 0.7071068
        wpose.orientation.z = 0.0
        wpose.orientation.w = 0.0
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.x = -0.96
        waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = self.right_arm.compute_cartesian_path(
            waypoints,  # waypoints to follow
            0.01,  # eef_step
            0.0)  # jump_threshold

        file_path = package_path + '/data/trajectory/tool/pick_approach_r_2.yaml'
        # dump the plan into yaml file
        with open(file_path, 'w') as file_save:
            yaml.dump(plan, file_save, default_flow_style=True)

        self.right_arm.execute(plan, wait=True)

        raw_input("wait for input h")  # h

        # move up
        waypoints = []
        wpose.position.z = 1.45
        waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = self.right_arm.compute_cartesian_path(
            waypoints,  # waypoints to follow
            0.01,  # eef_step
            0.0)  # jump_threshold

        file_path = package_path + '/data/trajectory/tool/pick_up_r_2.yaml'
        # dump the plan into yaml file
        with open(file_path, 'w') as file_save:
            yaml.dump(plan, file_save, default_flow_style=True)

        self.right_arm.execute(plan, wait=True)

        raw_input("wait for input c")  # c

        waypoints = []
        waypoints.append(copy.deepcopy(self.init_pose_r))

        (plan, fraction) = self.right_arm.compute_cartesian_path(
            waypoints,  # waypoints to follow
            0.01,  # eef_step
            0.0)  # jump_threshold

        file_path = package_path + '/data/trajectory/tool/pick_out_r_2.yaml'
        # dump the plan into yaml file
        with open(file_path, 'w') as file_save:
            yaml.dump(plan, file_save, default_flow_style=True)

        self.right_arm.execute(plan, wait=True)

    def right_place_2(self):
        rospack = rospkg.RosPack()
        package_path = rospack.get_path('drawing')

        waypoints = []
        wpose = self.right_arm.get_current_pose().pose
        wpose.position.x = -0.98
        wpose.position.y = 0.455
        wpose.position.z = 1.45
        wpose.orientation.x = 0.0  # -0.7071068
        wpose.orientation.y = 1.0  # 0.7071068
        wpose.orientation.z = 0.0
        wpose.orientation.w = 0.0
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.z = 1.40
        waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = self.right_arm.compute_cartesian_path(
            waypoints,  # waypoints to follow
            0.01,  # eef_step
            0.0)  # jump_threshold

        file_path = package_path + '/data/trajectory/tool/place_ready_r_2.yaml'
        # dump the plan into yaml file
        with open(file_path, 'w') as file_save:
            yaml.dump(plan, file_save, default_flow_style=True)

        self.right_arm.execute(plan, wait=True)

        raw_input("wait for input h")  # h

        # place
        waypoints = []
        wpose.position.z = 1.32
        waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = self.right_arm.compute_cartesian_path(
            waypoints,  # waypoints to follow
            0.01,  # eef_step
            0.0)  # jump_threshold

        file_path = package_path + '/data/trajectory/tool/place_down_r_2.yaml'
        # dump the plan into yaml file
        with open(file_path, 'w') as file_save:
            yaml.dump(plan, file_save, default_flow_style=True)

        self.right_arm.execute(plan, wait=True)

        raw_input("wait for input o")  # o

        # back, side, forward
        waypoints = []
        wpose.position.x = -0.8
        waypoints.append(copy.deepcopy(wpose))
        waypoints.append(copy.deepcopy(self.init_pose_r))

        (plan, fraction) = self.right_arm.compute_cartesian_path(
            waypoints,  # waypoints to follow
            0.01,  # eef_step
            0.0)  # jump_threshold

        file_path = package_path + '/data/trajectory/tool/place_out_r_2.yaml'
        # dump the plan into yaml file
        with open(file_path, 'w') as file_save:
            yaml.dump(plan, file_save, default_flow_style=True)

        self.right_arm.execute(plan, wait=True)


    def init_pose (self):
        init_r = [-1.5027168989181519, -2.0134006939330042, -2.965165440236227, -1.369880588059761, 0.800791323184967, 2.57310152053833]
        init_l = [1.5191028753863733, -1.1548146170428772, 2.9776430130004883, -1.773144384423727, -0.7987497488604944, 2.1262924671173096]
        # init_r = [-170*D2R, -120*D2R, -80*D2R, -70*D2R, 45*D2R, 140*D2R]
        # init_l = [170*D2R, -65*D2R, 80*D2R, -110*D2R, -45*D2R, 130*D2R]

        joint_goal = self.arm.get_current_joint_values()
        for i in range(6):
            if self.arm_num == 0: joint_goal[i] = init_r[i]
            if self.arm_num == 1: joint_goal[i] = init_l[i]
        self.arm.go(joint_goal, wait=True)

        if self.arm_num == 0:
            self.init_pose_r = self.right_arm.get_current_pose().pose
        elif self.arm_num == 1:
            self.init_pose_l = self.left_arm.get_current_pose().pose


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
        rospack = rospkg.RosPack()
        package_path = rospack.get_path('drawing')
        file_path = package_path + '/data/trajectory/tool/pick_approach_r_1.yaml'

        with open('', 'r') as file_open:
            loaded_plan = yaml.load(file_open)
            joints = loaded_plan.joint_trajectory.joint_names
            for joint in joints:
                print(joint)


if __name__ == '__main__':

    rospy.init_node('trajectory_loader', anonymous=True)

    rospack = rospkg.RosPack()
    package_path = rospack.get_path('drawing')
    file_path = package_path + '/data/trajectory/test/' + FILE_NAME

    right = TrajectoryLoader(file_path, 0)
    # left = TrajectoryLoader(file_path, 1)

    right.right_pick_1()
    right.right_place_1()

    right.right_pick_2()
    right.right_place_2()

    # right.left_pick_1()
    # right.left_place_1()
    #
    # right.left_pick_2()
    # right.left_place_2()
