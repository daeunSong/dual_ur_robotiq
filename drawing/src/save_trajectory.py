#!/usr/bin/env python

import yaml
import rospy
import rospkg
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import moveit_commander
from geometry_msgs.msg import Pose, Point
from std_msgs.msg import Int32, Bool

FILE_NAME = 'heart_path_'
FILE_COLORS = ['c', 'm', 'y', 'k']
FILE_NUMS = [10, 26, 2, 8]

class TrajeectorySaver:
    def __init__(self, file_path):
        self.trajectory_sub = rospy.Subscriber("/trajectory", RobotTrajectory, self.callback_traj)
        self.color_sub = rospy.Subscriber("/drawing_color", Point, self.callback_color)
        self.armNum_sub = rospy.Subscriber("/arm_number", Int32, self.callback_armNum)
        self.file_path_init = file_path
        self.file_path = file_path
        self.plan = None
        self.color = None
        self.arm_num = 0
        self.file_num = 0


    def callback_traj(self, msg):
        self.plan = msg
        if self.arm_num == 0:   # right
            self.file_path = self.file_path_init + 'r_' + self.color + '_' + str(self.file_num) + '.yaml'
        elif self.arm_num == 1: # left
            self.file_path = self.file_path_init + 'l_' + self.color + '_' + str(self.file_num) + '.yaml'
        self.file_num = self.file_num + 1

        # dump the plan into yaml file
        with open(self.file_path, 'w') as file_save:
            yaml.dump(self.plan, file_save, default_flow_style=True)

    def callback_color(self, msg):
        if msg.x == 0.0 and msg.y == 1.0 and msg.z == 1.0: # cyan
            self.color = 'c'
            self.file_num = 0
        elif msg.x == 1.0 and msg.y == 0.0 and msg.z == 1.0: # magenta
            self.color = 'm'
            self.file_num = 0
        elif msg.x == 1.0 and msg.y == 1.0 and msg.z == 0.0: # yellow
            self.color = 'y'
            self.file_num = 0
        else: # black
            self.color = 'k'
            self.file_num = 0
        print('message recieved: ' + self.color)

    def callback_armNum(self, msg):
        self.arm_num = msg.data


class TrajectoryLoader:
    def __init__(self, file_path):
        self.file_path_init = file_path
        self.file_path = file_path

        # publisher
        self.drawing_line_pub = rospy.Publisher('/ready_to_draw', Bool, queue_size=1)
        self.drawing_color_pub = rospy.Publisher('/drawing_color', Point, queue_size=1)

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

            for num in range(FILE_NUMS[i]):
                ready = Bool()
                self.file_path = self.file_path_init + 'r_' + color + '_' + str(num) + '.yaml'
                print('file path: ' + self.file_path)
                with open(self.file_path, 'r') as file_open:
                    print('loading ...')
                    loaded_plan = yaml.load(file_open)
                    # move to first pose
                    joint_goal = self.right_arm.get_current_joint_values()
                    for j, q in enumerate(loaded_plan.joint_trajectory.points[0].positions):
                        joint_goal[j] = loaded_plan.joint_trajectory.points[0].positions[j]
                    self.right_arm.go(joint_goal, wait=True)
                    # draw
                    ready.data = True
                    self.drawing_line_pub.publish(ready)
                    print('drawing ' + color + ' ' + str(num) + 'th stroke')
                    self.right_arm.execute(loaded_plan, wait=True)
                    ready.data = False
                    self.drawing_line_pub.publish(ready)


if __name__ == '__main__':

    rospy.init_node('trajectory_saver', anonymous=True)

    rospack = rospkg.RosPack()
    package_path = rospack.get_path('drawing')
    file_path = package_path + '/data/trajectory/' + FILE_NAME

    # Saver = TrajeectorySaver(file_path)
    Loader = TrajectoryLoader(file_path)
    Loader.load_traj()

    # Keep the program running
    rospy.spin()
