#!/usr/bin/env python

import sys

import math
import rospy
import rospkg
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from controller_manager_msgs.srv import SwitchControllerRequest, SwitchController
from controller_manager_msgs.srv import LoadControllerRequest, LoadController
from controller_manager_msgs.srv import ListControllers, ListControllersRequest
import geometry_msgs.msg as geometry_msgs
from std_msgs.msg import Char, Int32
from cartesian_control_msgs.msg import (
    FollowCartesianTrajectoryAction,
    FollowCartesianTrajectoryGoal,
    CartesianTrajectoryPoint,
)
import copy, yaml

import moveit_commander
import numpy as np
import matplotlib.pyplot as plt
import math
from scipy.optimize import minimize, rosen, rosen_der
import random

# Compatibility for python2 and python3
if sys.version_info[0] < 3:
    input = raw_input


FILE_NAME = "../data/input/sbr/actions_6.csv"
TARGET_SIZE = 0.25
RATIO = 500.0/500.0
# RATIO = 500.0/500.0
# RATIO = 334.0.0/500.0
# RATIO = 641.0/513.0

# If your robot description is created with a tf_prefix, those would have to be adapted
JOINT_NAMES = [
    "left_shoulder_pan_joint",
    "left_shoulder_lift_joint",
    "left_elbow_joint",
    "left_wrist_1_joint",
    "left_wrist_2_joint",
    "left_wrist_3_joint",
]

# All of those controllers can be used to execute joint-based trajectories.
# The scaled versions should be preferred over the non-scaled versions.
JOINT_TRAJECTORY_CONTROLLERS = [
    "scaled_pos_joint_traj_controller",
    "scaled_vel_joint_traj_controller",
    "pos_joint_traj_controller",
    "vel_joint_traj_controller",
    "forward_joint_traj_controller",
]

# All of those controllers can be used to execute Cartesian trajectories.
# The scaled versions should be preferred over the non-scaled versions.
CARTESIAN_TRAJECTORY_CONTROLLERS = [
    "pose_based_cartesian_traj_controller",
    "joint_based_cartesian_traj_controller",
    "forward_cartesian_traj_controller",
]

# We'll have to make sure that none of these controllers are running, as they will
# be conflicting with the joint trajectory controllers
CONFLICTING_CONTROLLERS = ["joint_group_vel_controller", "twist_controller"]


class TrajectoryClient:
    """Small trajectory client to test a joint trajectory"""

    def __init__(self):
        rospy.init_node("draw_left")

        self.state_sub = rospy.Subscriber('/left_arm/drawing_command', Int32, self.state_callback)
        self.state_pub = rospy.Publisher('/left_arm/drawing_state', Int32, queue_size=1)
        self.command = 0

        # drawing
        self.file_name = FILE_NAME
        self.drawing_num = 0
        self.colors = [2, 0]
        self.num_drawing = len(self.colors)

        # gripper
        self.gripper = rospy.Publisher('/left_gripper/gripper_left', Char, queue_size=1)

        timeout = rospy.Duration(5)
        self.switch_srv = rospy.ServiceProxy(
            "/left_arm/controller_manager/switch_controller", SwitchController
        )
        self.load_srv = rospy.ServiceProxy("/left_arm/controller_manager/load_controller", LoadController)
        self.list_srv = rospy.ServiceProxy("/left_arm/controller_manager/list_controllers", ListControllers)
        try:
            self.switch_srv.wait_for_service(timeout.to_sec())
        except rospy.exceptions.ROSException as err:
            rospy.logerr("Could not reach controller switch service. Msg: {}".format(err))
            sys.exit(-1)

        self.joint_trajectory_controller = JOINT_TRAJECTORY_CONTROLLERS[0]
        self.cartesian_trajectory_controller = CARTESIAN_TRAJECTORY_CONTROLLERS[0]
        self.trajectory_client = None
        self.init_client()

        # moveit
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.right_arm = moveit_commander.MoveGroupCommander("right_arm")
        self.left_arm = moveit_commander.MoveGroupCommander("left_arm")
        self.arms = moveit_commander.MoveGroupCommander("arms")
        self.arm = self.left_arm
        self.right_arm.set_end_effector_link("right_gripper_tool0")
        self.left_arm.set_end_effector_link("left_gripper_tool0")
        rospy.sleep(3)

        ### tool trajectories
        self.num_tool = self.num_drawing
        self.tool_pick_traj = []
        self.tool_place_traj = []
        self.load_tool_traj()
        self.goal = FollowJointTrajectoryGoal()
        self.goal.trajectory.joint_names = JOINT_NAMES

        ## drawing tarjectories
        self.drawing_traj_full = []
        # self.load_drawing_traj()

        rospy.loginfo("Loading Done")

    def init_client (self):
        # make sure the correct controller is loaded and activated
        self.switch_controller(self.joint_trajectory_controller)
        self.trajectory_client = actionlib.SimpleActionClient(
            "{}/follow_joint_trajectory".format(self.joint_trajectory_controller),
            FollowJointTrajectoryAction,
        )

        # Wait for action server to be ready
        timeout = rospy.Duration(5)
        if not self.trajectory_client.wait_for_server(timeout):
            rospy.logerr("Could not reach controller action server.")
            sys.exit(-1)
        rospy.loginfo("Init client")

    def state_callback (self, msg):
        self.command = msg.data

    def tool_pick (self, tool_num):
        action = Char()
        pick_traj = self.tool_pick_traj[tool_num]
        # approach
        self.goal.trajectory.points = pick_traj[0]
        self.trajectory_client.send_goal(self.goal)
        self.trajectory_client.wait_for_result()
        result = self.trajectory_client.get_result()
        action.data = 104
        self.gripper.publish(action)
        rospy.sleep(1)
        # up
        self.goal.trajectory.points = pick_traj[1]
        self.trajectory_client.send_goal(self.goal)
        self.trajectory_client.wait_for_result()
        result = self.trajectory_client.get_result()
        action.data = 99
        self.gripper.publish(action)
        rospy.sleep(1.5)
        # out
        self.goal.trajectory.points = pick_traj[2]
        self.trajectory_client.send_goal(self.goal)
        self.trajectory_client.wait_for_result()
        result = self.trajectory_client.get_result()

    def tool_place (self, tool_num):
        action = Char()
        place_traj = self.tool_place_traj[tool_num]
        # approach
        self.goal.trajectory.points = place_traj[0]
        self.trajectory_client.send_goal(self.goal)
        self.trajectory_client.wait_for_result()
        result = self.trajectory_client.get_result()
        action.data = 104
        self.gripper.publish(action)
        rospy.sleep(1)
        # up
        self.goal.trajectory.points = place_traj[1]
        self.trajectory_client.send_goal(self.goal)
        self.trajectory_client.wait_for_result()
        result = self.trajectory_client.get_result()
        action.data = 111
        self.gripper.publish(action)
        rospy.sleep(1)
        # out
        self.goal.trajectory.points = place_traj[2]
        self.trajectory_client.send_goal(self.goal)
        self.trajectory_client.wait_for_result()
        result = self.trajectory_client.get_result()

    def load_tool_traj (self):
        rospack = rospkg.RosPack()
        file_path_init = rospack.get_path('drawing') + '/data/trajectory/tool/'
        file_num = 1
        pick = ['approach', 'up', 'out']
        place = ['ready', 'down', 'out']

        while (file_num <= self.num_tool):
            pick_traj = [] # pick
            for stage in pick:
                file_path = file_path_init + 'pick_'+ stage + '_l_'+str(file_num)+'.yaml'
                with open(file_path, 'r') as file_open:
                    loaded_plan = yaml.load(file_open)
                    pick_traj.append(copy.deepcopy(loaded_plan.joint_trajectory.points))
            self.tool_pick_traj.append(copy.deepcopy(pick_traj))

            place_traj = [] # place
            for stage in place:
                file_path = file_path_init + 'place_' + stage + '_l_' + str(file_num)+'.yaml'
                with open(file_path, 'r') as file_open:
                    loaded_plan = yaml.load(file_open)
                    place_traj.append(copy.deepcopy(loaded_plan.joint_trajectory.points))
            self.tool_place_traj.append(copy.deepcopy(place_traj))
            file_num += 1

        rospy.loginfo("Done reading pre-defined trajectories")

    # def draw (self, drawing_num):
    #     drawing_traj = self.drawing_traj_full[drawing_num]
    #     for i in range(3):
    #         self.goal.trajectory.points = drawing_traj[i]
    #         self.trajectory_client.send_goal(self.goal)
    #         self.trajectory_client.wait_for_result()
    #         result = self.trajectory_client.get_result()
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

    def set_pose_lin (self, p):
        waypoints = []
        waypoints.append(copy.deepcopy(self.set_pose(p)))
        (plan, fraction) = self.arm.compute_cartesian_path(
            waypoints,  # waypoints to follow
            0.01,  # eef_step
            0.0)  # jump_threshold
        result = self.arm.execute(plan, wait=True)

    def draw (self, s):
        # move to ready pose
        p = s[0] + [1.34, 0.7071068, 0.7071068, 0.0, 0.0]
        self.set_pose_lin(p)

        # draw strokes
        waypoints = []
        waypoints.append(copy.deepcopy(self.set_pose(p)))
        for p_ in s:
            p = p_ + [1.32, 0.7071068, 0.7071068, 0.0, 0.0]
            waypoints.append(copy.deepcopy(self.set_pose(p)))
        p = s[-1] + [1.35, 0.7071068, 0.7071068, 0.0, 0.0]
        waypoints.append(copy.deepcopy(self.set_pose(p)))

        (plan, fraction) = self.arm.compute_cartesian_path(
            waypoints,  # waypoints to follow
            0.01,  # eef_step
            0.0)  # jump_threshold
        result = self.arm.execute(plan, wait=True)



    # def load_drawing_traj (self):
    #     rospack = rospkg.RosPack()
    #     # file_path_init = rospack.get_path('drawing') + '/data/trajectory/drawing/Starry_Night/'
    #     file_path_init = rospack.get_path('drawing') + '/data/trajectory/drawing/'
    #     for color in self.colors:
    #         drawing_traj = []
    #         for i in range(3):
    #             file_path = file_path_init + self.file_name + 'l_' + color + '_' + str(i)+'.yaml'
    #             with open(file_path, 'r') as file_open:
    #                 loaded_plan = yaml.load(file_open)
    #                 drawing_traj.append(copy.deepcopy(loaded_plan.joint_trajectory.points))
    #         self.drawing_traj_full.append(copy.deepcopy(drawing_traj))
    #
    #     rospy.loginfo("Done reading pre-defined trajectories")

    def send_joint_trajectory(self, goal):
        """Creates a trajectory and sends it using the selected action server"""

        # make sure the correct controller is loaded and activated
        self.switch_controller(self.joint_trajectory_controller)
        trajectory_client = actionlib.SimpleActionClient(
            "{}/follow_joint_trajectory".format(self.joint_trajectory_controller),
            FollowJointTrajectoryAction,
        )

        # Wait for action server to be ready
        timeout = rospy.Duration(5)
        if not trajectory_client.wait_for_server(timeout):
            rospy.logerr("Could not reach controller action server.")
            sys.exit(-1)

        ########################################### get goal as input
        # # Create and fill trajectory goal
        # goal = FollowJointTrajectoryGoal()
        # goal.trajectory.joint_names = JOINT_NAMES
        #
        # # The following list are arbitrary positions
        # # Change to your own needs if desired
        # position_list = [[-170*D2R, -120*D2R, -80*D2R, -70*D2R, 45*D2R, 140*D2R]]
        # # position_list.append([-174 * D2R, -104 * D2R, -93 * D2R, -1 * D2R, 69 * D2R, 140 * D2R])
        # # position_list.append([-174 * D2R, -104 * D2R, -93 * D2R, -1 * D2R, 69 * D2R, 150 * D2R])
        # duration_list = [5.0]
        # # duration_list = [5.0, 10.0, 15.0]
        # for i, position in enumerate(position_list):
        #     point = JointTrajectoryPoint()
        #     point.positions = position
        #     point.time_from_start = rospy.Duration(duration_list[i])
        #     goal.trajectory.points.append(point)

        trajectory_client.send_goal(goal)
        trajectory_client.wait_for_result()

        result = trajectory_client.get_result()
        rospy.loginfo("Trajectory execution finished in state {}".format(result.error_code))

    def switch_controller(self, target_controller):
        """Activates the desired controller and stops all others from the predefined list above"""
        other_controllers = (
            JOINT_TRAJECTORY_CONTROLLERS
            + CARTESIAN_TRAJECTORY_CONTROLLERS
            + CONFLICTING_CONTROLLERS
        )

        other_controllers.remove(target_controller)

        srv = ListControllersRequest()
        response = self.list_srv(srv)
        for controller in response.controller:
            if controller.name == target_controller and controller.state == "running":
                return

        srv = LoadControllerRequest()
        srv.name = target_controller
        self.load_srv(srv)

        srv = SwitchControllerRequest()
        srv.stop_controllers = other_controllers
        srv.start_controllers = [target_controller]
        srv.strictness = SwitchControllerRequest.BEST_EFFORT
        self.switch_srv(srv)


class Point(object):
    def __init__(self, x=0, y=0):
        self.x = x
        self.y = y

    def random(self, min= 0, max= 1):
        self.x = random.uniform(min,max)
        self.y = random.uniform(min,max)


class QuadBezier(object):
    def __init__(self, p0x= 0, p0y= 0, p1x= 0, p1y= 0, p2x= 0, p2y= 0):
        self.p0 = Point(p0x, p0y)
        self.p1 = Point(p1x, p1y)
        self.p2 = Point(p2x, p2y)
        self.obstacles = []

    def random(self,min= 0, max= 1):
        'Create a random quadratic Bezier curve within [min, max] limits. Default [0,1].'
        self.p0.random(min, max)
        self.p1.random(min, max)
        self.p2.random(min, max)

    def max_k(self, granuality=100):
        'Calculate maximal curvature of the quadratic Bezier curve.'
        k = 0
        for t in range(0, granuality):
            t = t / granuality
            x_d = 2 * (t - 1)*(self.p1.x - self.p0.x) + 2 * t * (self.p2.x - self.p1.x)
            y_d = 2 * (t - 1)*(self.p1.y - self.p0.y) + 2 * t * (self.p2.y - self.p1.y)
            x_dd = 2 * (self.p2.x - 2 * self.p1.x + self.p0.x)
            y_dd = 2 * (self.p2.y - 2 * self.p1.y + self.p0.y)
            k = max(k,abs(x_d*y_dd - y_d*x_dd)/math.pow(x_d**2 + y_d**2, 3/2))
        return k

    def calc_curve(self, granuality=10):
        'Calculate the quadratic Bezier curve with the given granuality.'
        B_x = []
        B_y = []
        for t in range(0, granuality):
            t = t / granuality
            x = self.p1.x + (1 - t)**2 * (self.p0.x-self.p1.x) + t**2 * (self.p2.x - self.p1.x)
            y = self.p1.y + (1 - t)**2 * (self.p0.y-self.p1.y) + t**2 * (self.p2.y - self.p1.y)
            B_x.append(x)
            B_y.append(y)
        return [B_x, B_y]

    def plot(self, granuality=100):
        'Plot the quadratic Bezier curve.'
        B = self.calc_curve(granuality)
        plt.plot(B[0], B[1])
        # plt.scatter([self.p0.x,self.p1.x,self.p2.x], [self.p0.y,self.p1.y,self.p2.y])
        for i in range(len(self.obstacles)):
            plt.gcf().gca().add_artist(plt.Circle((self.obstacles[i][0].x, self.obstacles[i][0].y), self.obstacles[i][1], color='r'))
        plt.axis('equal')
        # plt.ion()
        # plt.show()
        return B

    def arc_len(self, granuality=1000):
        'Calculate the arc-length of the quadratic Bezier curve.'
        B = self.calc_curve(granuality=granuality)
        a_l = 0
        for i in range(1,len(B[0])):
            a_l += math.sqrt((B[0][i]-B[0][i-1])**2 + (B[1][i]-B[1][i-1])**2)
        return a_l

    def optimize_k(self, granuality= 100, obs= True):
        'Optimize the quadratic Bezier curve to minimize the curvature. By setting obs=False, ignore the obstacles.'
        x0 = [0.0, 0.0]
        res = minimize(self.optimizer_k, x0, args= (granuality, obs), method='Nelder-Mead', tol=1e-7)
        self.p1.x = self.p1.x + res.x[0]
        self.p1.y = self.p1.y + res.x[1]

    def optimizer_k(self,x, *args):
        'Curvature optimizer function.'
        granuality = args[0]
        obs = args[1]
        o = QuadBezier()
        o.p0 = self.p0
        o.p1.x = self.p1.x+x[0]
        o.p1.y = self.p1.y+x[1]
        o.p2 = self.p2
        penalty = 0

        if obs:
            B = o.calc_curve(granuality)
            for i in range(len(B[0])):
                for j in range(len(self.obstacles)):
                    d = math.sqrt((B[0][i] - self.obstacles[j][0].x)**2 + (B[1][i] - self.obstacles[j][0].y)**2)
                    if d<self.obstacles[j][1]:
                        penalty += (self.obstacles[j][1]-d)*100
        return o.max_k(granuality) + penalty

    def optimize_l(self, granuality= 100, obs= True):
        'Optimize the quadratic Bezier curve to minimize the arc-length. By setting obs=False, ignore the obstacles.'
        x0 = [0.0, 0.0]
        res = minimize(self.optimizer_l, x0, args=(granuality, obs), method='Nelder-Mead', tol=1e-7)
        self.p1.x = self.p1.x + res.x[0]
        self.p1.y = self.p1.y + res.x[1]

    def optimizer_l(self,x, *args):
        'Arc-length optimizer function.'
        granuality = args[0]
        obs = args[1]
        o = QuadBezier()
        o.p0 = self.p0
        o.p1.x = self.p1.x+x[0]
        o.p1.y = self.p1.y+x[1]
        o.p2 = self.p2

        penalty = 0
        if obs:
            B = o.calc_curve(granuality)
            for i in range(len(B[0])):
                for j in range(len(self.obstacles)):
                    d = math.sqrt((B[0][i] - self.obstacles[j][0].x)**2 + (B[1][i] - self.obstacles[j][0].y)**2)
                    if d<self.obstacles[j][1]:
                        penalty += (self.obstacles[j][1]-d)*100

        return o.arc_len(granuality) + penalty

    def optimize(self, granuality=100, obs=True, l_multiplier=0.5, k_multiplier=0.5):
        """
        Optimize the quadratic Bezier curve to simultaniously minimize the arc-lenght and the curvature.
        Setting obs=False ignores the obstacles. l_multiplier and k_multiplier multiplies
        the outputs of their respective optimizer functions.
        """
        x0 = [0.0, 0.0]
        res = minimize(self.optimizer, x0, args=(granuality, obs, l_multiplier, k_multiplier), method='Nelder-Mead', tol=1e-7)
        self.p1.x = self.p1.x + res.x[0]
        self.p1.y = self.p1.y + res.x[1]

    def optimizer(self,x,*args):
        'Optimizer function of the arc-length and curvature simultanious optimization.'
        granuality = args[0]
        obs = args[1]
        l_multiplier = args[2]
        k_multiplier = args[3]

        return self.optimizer_l(x, granuality, obs) * l_multiplier + self.optimizer_k(x, granuality, obs) * k_multiplier

    def add_obstacle(self, x=0, y=0, radius=0):
        'Add an obstacle to the quadratic Bezier curve.'
        self.obstacles.append([Point(x,y), radius])

    def add_random_obstacle(self, min_x= 1, max_x= 0, min_y=1, max_y=0, min_radius=0.3, max_radius = 0.0):
        """Add a random obstacle to the quadratic Bezier curve. The obstacle will not cover the p0 and p2 points
        of the Bezier curve.
        """
        radius = random.uniform(min_radius,max_radius)

        d = 0
        x = 0
        y = 0
        while d<radius:
            x = random.uniform(min_x,max_x)
            y = random.uniform(min_y,max_y)
            d1 = math.sqrt((x - self.p0.x)**2 + (y - self.p0.y)**2)
            d2 = math.sqrt((x - self.p2.x) ** 2 + (y - self.p2.y) ** 2)
            d = min(d1,d2)

        self.obstacles.append([Point(x, y), radius])

    def clear(self):
        'Re-initialize the curve.'
        self.__init__()


def read_file (file_name):
    f = open(file_name)
    strokes = []
    strokes_by_colors = []
    i = 0.0

    while True:
        line = f.readline()
        if not line or line == "":
            strokes_by_colors.append(strokes)
            break
        ctrl_points = []
        x0, y0, x1, y1, x2, y2, _, _, _, _, c, _, _= list(map(float,line.split(",")))
        x0 = (x0 - 0.5) * RATIO * TARGET_SIZE - 0.55
        y0 = (y0 - 0.5) * TARGET_SIZE
        x1 = (x1 - 0.5) * RATIO * TARGET_SIZE - 0.55
        y1 = (y1 - 0.5) * TARGET_SIZE
        x2 = (x2 - 0.5) * RATIO * TARGET_SIZE - 0.55
        y2 = (y2 - 0.5) * TARGET_SIZE

        if i != c: # end of color
            strokes_by_colors.append(strokes)
            strokes = []
            i += 1

        s_ = QuadBezier(x0, y0, x1, y1, x2, y2)
        s = s_.plot() # stroke in [xs, ys]
        strokes.append(s)

    print("Read file Done")

    return strokes_by_colors

    # return np.array(strokes)
    # print("done")
    # plt.show()

if __name__ == '__main__':
    file_name = FILE_NAME

    client = TrajectoryClient()
    rospy.sleep(3)

    strokes_by_colors = read_file(file_name)
    strokes_by_colors = [strokes_by_colors[client.colors[0],strokes_by_colors[client.colors[1]]]]

    # client.tool_pick(0)
    # client.tool_place(0)
    # client.tool_pick(1)
    # client.tool_place(1)
    #
    state = Int32()

    # start drawing
    for i in range(client.num_drawing):
        # wait for right to done drawing
        while client.command == 0:
            state.data = 0

        # pick up the tool
        client.tool_pick(i)
        print("tool pick")
        # draw
        state.data = 1
        print("draw")
        client.state_pub.publish(state)
        for s in strokes_by_colors[i]:
            client.draw(s)
        rospy.sleep(3)
        # place the tool
        print("tool place")
        state.data = 0
        client.command = 0
        client.state_pub.publish(state)
        client.tool_place(i)