# Tiago_arm_effort_controller

This project consists of a ROS plugin to deploy on a TIAGo (from pal-robotics) in order to control its arm in Pose and Effort (following the Dual Control Law described in "*Modeling, Identification and Control of Robots*,  Ed. by W Khalil and E Dombre) using a force controller.

Bellow, you will find the basic steps to work with this plugin. For setting this package up and for more informations refer to the [wiki](https://github.com/aip-primeca-occitanie/tiago_arm_effort_controller/wiki) !

> :bulb: For all commands below, the name of the catkin workspace is `tiago_ws` and the name of the robot `tiago-155c`. Adjust to your needs.

## in Simulation

> :warning: The simulation is not accurate !

Terminal :one::
```
cd ~/tiago_ws
catkin build
source devel/setup.bash
roslaunch tiago_gazebo tiago_gazebo.launch robot:=steel
```

Terminal :two::
```
cd ~/tiago_ws
source devel/setup.bash
```
```
rosservice call /controller_manager/switch_controller "start_controllers:
- ''
stop_controllers:
- 'arm_controller'
strictness: 0" 
```
```
roslaunch tiago_arm_effort_controller tiago_arm_effort_controller.launch robot:=tiago end_effector:=pal-gripper simulation:=true
```

## on the TIAGo

:computer: pal@development:
```
cd ~/tiago_ws
catkin build
rosrun pal_deploy deploy.py tiago-155c
```
:robot: pal@tiago-155c:

> :bulb: to connect to the TIAGo's terminal:
> ```
> ssh pal@tiago-155c
> ```

```
pal_restart_deployer
```
```
rosservice call /controller_manager/switch_controller "start_controllers:
- ''
stop_controllers:
- 'arm_controller'
strictness: 0" 
```
```
roslaunch tiago_arm_effort_controller tiago_arm_effort_controller.launch robot:=tiago end_effector:=pal-gripper
```
