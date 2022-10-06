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

# Compatibility for python2 and python3
if sys.version_info[0] < 3:
    input = raw_input

FILE_NAME = 'sbr_1_path_'
STROKE_NUM = [1639, 729]

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
        self.colors = ['1', '3']
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

        ### tool trajectories
        self.num_tool = self.num_drawing
        self.tool_pick_traj = []
        self.tool_place_traj = []
        self.load_tool_traj()
        self.goal = FollowJointTrajectoryGoal()
        self.goal.trajectory.joint_names = JOINT_NAMES

        ## drawing tarjectories
        self.drawing_traj_full = []
        self.load_drawing_traj()

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

        rospy.loginfo("Done reading pre-defined tool trajectories")

    def draw (self, drawing_num):
        drawing_traj = self.drawing_traj_full[drawing_num]
        for i in range(len(drawing_traj)):
            self.goal.trajectory.points = drawing_traj[i]
            self.trajectory_client.send_goal(self.goal)
            self.trajectory_client.wait_for_result()
            result = self.trajectory_client.get_result()

    def load_drawing_traj (self):
        rospack = rospkg.RosPack()
        # file_path_init = rospack.get_path('drawing') + '/data/trajectory/drawing/Starry_Night/'
        file_path_init = rospack.get_path('drawing') + '/data/trajectory/drawing/'
        j = 0
        for color in self.colors:
            drawing_traj = []
            for i in range(STROKE_NUM[j]):
                file_path = file_path_init + self.file_name + 'l_' + color + '_' + str(i)+'.yaml'
                with open(file_path, 'r') as file_open:
                    loaded_plan = yaml.load(file_open)
                    drawing_traj.append(copy.deepcopy(loaded_plan.joint_trajectory.points))
            self.drawing_traj_full.append(copy.deepcopy(drawing_traj))
            j += 1

        rospy.loginfo("Done reading pre-defined drawing trajectories")

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


if __name__ == "__main__":
    client = TrajectoryClient()
    rospy.sleep(3)

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
        print("draw")
        state.data = 1
        client.state_pub.publish(state)
        client.draw(i)
        rospy.sleep(3)
        # place the tool
        print("tool place")
        state.data = 0
        client.command = 0
        client.state_pub.publish(state)
        client.tool_place(i)
