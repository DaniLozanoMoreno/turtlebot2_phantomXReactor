# turtlebot2_phantomXReactor

Author: Daniel Lozano Moreno

*Gazebo Kinetic.*

Proyect created for the final degree project in robotics engineering at the University of Alicante (UA), promotion of 2021-2022. Proyect Name: Control y simulación en ROS de un PhatomX Reactor Arm en cooperación con un TurtleBot2 (Control and simulation in ROS of a PhatomX Reactor Arm in cooperation with a TurtleBot2).

These packages allow control of a PhantomX Reactor Arm in cooperation with a TurtleBot in gazebo simulation and in real. A MoveIt configuration and a simple Software architecture with trajectory planners for the arm control are offered.

* **turtlebot_arm_desciption**: robot model
* **turtlebot_arm_gazebo**: gazebo simulations
* **turtlebot_arm_moveit_config**: moveit configuration
* **turtlebot_arm_controllers**: control arquitecture

To use the real robot is necessary the arbotix_ros repository: https://github.com/vanadiumlabs/arbotix_ros

The documentations offered are in Spanish for now. Read it carefully to know how to use the repository:

* Manual de laboratorio del PhantomX Reactor Arm: explain how to use the real robot and gazebo simulation with this repository.
* Guia de uso del paquete arbotix_ros: explain how to configure the arbotix ros board (whish the robot works).
* Control y simulación en ROS de un PhantomX Reactor Arm en cooperacion con un TurtleBot2: explain how the repository has been developed.

# turtlebot_arm_description

Has been created two robots models:
* **phantomx_reactor**: arm plant and transmissions.
* **turtlebot_arm**: arm plant and transmissions and turtleBot2 model. 

PhantomX Reactor has his own repository in https://github.com/RobotnikAutomation/phantomx_reactor_arm , but it has some errors that don't allow to work with the turtlebot model. turlebot2_phantomXReactor respository works regardless of this repository.

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

To load models in Gazebo execute:

```
roslaunch turtlebot_arm_gazebo phantomx_reactor_gazebo.launch
```
```
roslaunch turtlebot_arm_gazebo turtlebot_arm_gazebo.launch
```

