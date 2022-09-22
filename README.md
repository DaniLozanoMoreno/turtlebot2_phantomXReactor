# turtlebot2_phantomXReactor

Author: Daniel Lozano Moreno

*Gazebo Kinetic.*

Proyect created for the final degree project in robotics engineering at the University of Alicante (UA), promotion of 2021-2022. Proyect Name: Control y simulación en ROS de un PhatomX Reactor Arm en cooperación con un TurtleBot2 (Control and simulation in ROS of a PhatomX Reactor Arm in cooperation with a TurtleBot2).

These packages allow the control of a PhantomX Reactor Arm in cooperation with a TurtleBot in gazebo simulation and in real. A MoveIt configuration and a simple Software architecture with trajectory planners for the arm control are offered.

* **turtlebot_arm_desciption**: robot model
* **turtlebot_arm_gazebo**: gazebo simulations
* **turtlebot_arm_moveit_config**: moveit configuration
* **turtlebot_arm_controllers**: control arquitecture

To use the real robot is necessary the arbotix_ros repository: https://github.com/vanadiumlabs/arbotix_ros

The documentations offered are in Spanish for now. Read it carefully to know how to use the repository:

* Manual de laboratorio del PhantomX Reactor Arm: explain how to use the real robot and gazebo simulation with this repository.
* Guia de uso del paquete arbotix_ros: explain how to configure the arbotix ros board (with whom the robot works).
* Control y simulación en ROS de un PhantomX Reactor Arm en cooperacion con un TurtleBot2: explain how the repository has been developed.

# turtlebot_arm_description

Has been created two robots models:
* **phantomx_reactor**: arm plant and transmissions.
* **turtlebot_arm**: arm plant, transmissions and turtleBot2 model. 

PhantomX Reactor has his own repository in https://github.com/RobotnikAutomation/phantomx_reactor_arm , but it has some errors that doesn't allow to work with the turtlebot model. turlebot2_phantomXReactor respository works regardless of this repository.

Descriptions can be loaded executing:
```
roslaunch turtlebot_arm_description phantomx_reactor_load_description.launch
```
```
roslaunch turtlebot_arm_description turtlebot_arm_load_description.launch
```
To load them in rviz:
```
roslaunch turtlebot_arm_description phantomx_reactor_rviz_demo.launch
```
```
roslaunch turtlebot_arm_description turtlebot_arm_rviz_demo.launch
```

# turtlebot_arm_gazebo

Robots models can be simulated in Gazebo. 

To load the robots models in Gazebo execute:

```
roslaunch turtlebot_arm_gazebo phantomx_reactor_gazebo.launch
```
```
roslaunch turtlebot_arm_gazebo turtlebot_arm_gazebo.launch
```
A teleoperating turtlebot_arm demo is offered executing:

```
*To load robot model, world and arm teleop controller (needed rqt_joint_trajectory_controller repository)*
roslaunch turtlebot_arm_gazebo turtlebot_arm_teleop_demo_gazebo.launch
```
```
*To load turtlebot mobile base teleop controller*
roslaunch turtlebot_teleop keyboard_teleop.launch
```
turtlebot_arm_multirobot_gazebo is for load several turtlebot_arm simultaneously. For now it doesn't work.

# turtlebot_arm_controller and real robot

The real PhantomXReactor works with an ArbotiX microcontroller, which commands directly to the arm servos. The ArbotiX is programmed with Arduino and for communication with ros, it need to be programmed with a driver. The robots in the robotic laboratory of UA are programmed with it. Otherwise, you should follow the steps described in the section "Setting up the Arbotix-M board" in https://github.com/RobotnikAutomation/phantomx_reactor_arm .

In ros, is needed the arbotix_ros driver to the comunication: https://github.com/vanadiumlabs/arbotix_ros . It allow to configurate the control mode of the arm servos. To know how to use this driver to comunicate with the real robot, read the *Guia de uso del paquete arbotix_ros* document. If you want to make a trajectory control with **trajectory_msgs/JointTrajectory** msgs type, you need to follow this steps:

1. Load phantomx_reactor_arm description.
2. Create a config.yaml like turtlebot2_phantomXReactor/turtlebot_arm_controller/config/config.yaml.
3. Load arbotix_driver like turtlebot2_phantomXReactor/turtlebot_arm_controller/launch/arbotix_phantomx_reactor_arm_wrist.launch.

Also, a simple control architecture has been created: phantomx_reactor.py. It offers trajectory plannificators and executers that can be programmed in a main. *Manual de laboratorio del PhantomX Reactor Arm* document offers a lot of exemples to know hot to use it.




 
