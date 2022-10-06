```commandline
roscore

roslaunch dual_ur_robotiq_description upload_grippers.launch    
roslaunch dual_ur_robotiq_description bringup_robot.launch    

roslaunch robot_moveit_config robot_moveit_planning_execution.launch sim:=false     
roslaunch robot_moveit_config moveit_rviz.launch config:=true   

```

### Saving the trajectory
```
rosrun drawing drawing_test
rosrun drawing save_trajectory.py
```

### Executing the drawing
```
rosrun drawing drawing_manager.py
rosrun drawing draw_right.py __ns:=/right_arm
rosrun drawing draw_left.py __ns:=/left_arm
```

### Note for myself
- dual_ur_robotiq_description/urdf/robot.urdf.xarcro
```
  <joint name="right_arm_joint" type="fixed">
    <parent link="stand" />
    <child link = "right_base_link" />
    <origin xyz="0 0.35 1.8" rpy="${PI/4*3} 0.028 ${PI}" />
  </joint>

  <joint name="left_arm_joint" type="fixed">
    <parent link="stand" />
    <child link = "left_base_link" />
    <origin xyz="0 -0.35 1.803" rpy="${(PI/4*3)+0.003} -0.005 0" />
  </joint>
```
- robotiq-3f-gripper_articulated_macro.xacro
```
  <origin xyz="0.0 0.168 0.0" rpy="0 0 1.5707"/>
```


### Simple Gripper Command
Run the gripper controller
```commandline
roscore
roslaunch dual_ur_robotiq_description upload_grippers.launch 
```

- Always activate the gripper first
    ```commandline
    rostopic pub /right_gripper/gripper_right std_msgs/Char "data: 97"
    rostopic pub /left_gripper/gripper_left std_msgs/Char "data: 97"
    ```
  Sending the character `a` in integer.


- Gripper Open and Close
  ```commandline
  rostopic pub /right_gripper/gripper_right std_msgs/Char "data: 111"
  rostopic pub /right_gripper/gripper_right std_msgs/Char "data: 104"
  rostopic pub /right_gripper/gripper_right std_msgs/Char "data: 99"
  ```
  Sending `o`, `h`, and `c` for opening, half-opening, and closing respectively. 
