#!/usr/bin/env python

import yaml
import rospy
import rospkg
from moveit_msgs.msg import RobotTrajectory
from geometry_msgs.msg import Pose, Point
from std_msgs.msg import Int32, Bool
import time

# FILE_NAME = 'heart_path_'
FILE_NAME = 'Starry_Night_path_'
# FILE_NAME = 'bigben_path_'
# FILE_NAME = 'dog_path_'
# FILE_NAME = 'red_flower_path_'
# FILE_NAME = 'bigben_1_path_'
# FILE_NAME = 'sunflower_path_'

class TrajeectorySaver:
    def __init__(self, file_path):
        self.trajectory_sub = rospy.Subscriber("/trajectory", RobotTrajectory, self.callback_traj)
        self.color_sub = rospy.Subscriber("/drawing_color", Point, self.callback_color)
        self.arm_num_sub = rospy.Subscriber("/arm_number", Int32, self.callback_arm_num)
        self.file_path_init = file_path
        self.file_path = file_path
        self.plan = None
        self.color = 'k'
        self.arm_num = 0
        self.file_num = 0
        self.start = time.time()

    def callback_traj(self, msg):
        print('trajectory recieved')
        cur_time = time.time()
        print(cur_time - self.start)

        self.plan = msg
        if self.arm_num == 0:   # right
            self.file_path = self.file_path_init + 'r_' + self.color + '_' + str(self.file_num) + '.yaml'
        elif self.arm_num == 1: # left
            self.file_path = self.file_path_init + 'l_' + self.color + '_' + str(self.file_num) + '.yaml'
        elif self.arm_num == -1: # bi
            self.file_path = self.file_path_init + 'bimanual_' + str(self.file_num) + '.yaml'

        self.file_num = self.file_num + 1

        # dump the plan into yaml file
        with open(self.file_path, 'w') as file_save:
            yaml.dump(self.plan, file_save, default_flow_style=True)

    def callback_color(self, msg):
        if self.arm_num != -1:
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

    def callback_arm_num(self, msg):
        self.arm_num = msg.data
        self.file_num = 0

if __name__ == '__main__':

    rospy.init_node('trajectory_saver', anonymous=True)

    rospack = rospkg.RosPack()
    package_path = rospack.get_path('drawing')
    file_path = package_path + '/data/trajectory/drawing/' + FILE_NAME

    Saver = TrajeectorySaver(file_path)

    # Keep the program running
    rospy.spin()
