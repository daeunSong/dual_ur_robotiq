#!/usr/bin/env python

import yaml
import rospy
import rospkg
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import moveit_commander
from geometry_msgs.msg import Pose, Point

FILE_NAME = 'heart_path_'
FILE_COLORS = ['c', 'm', 'y', 'k']
FILE_NUMS = [1, 2, 1, 1]

class TrajeectorySaver:
    def __init__(self, file_path):
        self.trajectory_sub = rospy.Subscriber("/trajectory", RobotTrajectory, self.callback_traj)
        self.color_sub = rospy.Subscriber("/drawing_color", Point, self.callback_color)
        self.file_path_init = file_path
        self.file_path = file_path
        self.plan = None
        self.color = None
        self.num = 0

    def callback_traj(self, msg):
        self.plan = msg
        self.file_path = self.file_path_init + self.color + '_' + str(self.num) + '.yaml'
        self.num = self.num + 1

        # dump the plan into yaml file
        with open(self.file_path, 'w') as file_save:
            yaml.dump(self.plan, file_save, default_flow_style=True)

    def callback_color(self, msg):
        if msg.x == 0.0 and msg.y == 1.0 and msg.z == 1.0: # cyan
            self.color = 'c'
            self.num = 0
        elif msg.x == 1.0 and msg.y == 0.0 and msg.z == 1.0: # magenta
            self.color = 'm'
            self.num = 0
        elif msg.x == 1.0 and msg.y == 1.0 and msg.z == 0.0: # yellow
            self.color = 'y'
            self.num = 0
        else: # black
            self.color = 'k'
            self.num = 0
        print('message recieved: ' + self.color)

class TrajectoryLoader:
    def __init__(self, file_path):
        self.file_path_init = file_path
        self.file_path = file_path

        # moveit
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.right_arm = moveit_commander.MoveGroupCommander("left_arm")
        self.right_gripper = moveit_commander.MoveGroupCommander("left_gripper")
        self.left_arm = moveit_commander.MoveGroupCommander("right_arm")
        self.left_gripper = moveit_commander.MoveGroupCommander("right_arm")
        self.right_arm.set_end_effector_link("left_gripper_tool0")
        self.right_arm.set_end_effector_link("right_gripper_tool0")

    def load_traj (self):
        for i, color in enumerate(FILE_COLORS):
            for num in range(FILE_NUMS[i]):
                self.file_path = self.file_path_init + color + '_' + str(num) + '.yaml'
                print('file path: ' + self.file_path)
                with open(self.file_path, 'r') as file_open:
                    print('loading ...')
                    loaded_plan = yaml.load(file_open)
                    print('drawing ' + color + ' ' + str(num) + 'th stroke')
                    self.right_arm.execute(loaded_plan, wait=True)


if __name__ == '__main__':

    rospy.init_node('trajectory_saver', anonymous=True)

    rospack = rospkg.RosPack()
    package_path = rospack.get_path('drawing')
    file_path = package_path + '/data/trajectory/' + FILE_NAME

    Saver = TrajeectorySaver(file_path)
    # Loader = TrajectoryLoader(file_path)
    # Loader.load_traj()

    # Keep the program running
    rospy.spin()
