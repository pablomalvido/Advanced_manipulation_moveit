# MOVEIT EXTENSION WITH ADVANCED MANIPULATION FUNCTIONS

The advanced\_manipulation\_pkg presents a set of functions that extend the current MoveIt capabilities, to perform advanced manipulation. This functions include: Automatic tool changing, single and dual-arm motion with full control on the EEF cartesian speed and acceleration, and several types of dual-arm synchronized movements.

These functions are presented in the following research article: *Malvido Fresnillo, P., Vasudevan, S., Mohammed, W. M., Martinez Lastra, J. L., & Perez Garcia, J. A. (2023). Extending the motion planning framework—MoveIt with advanced manipulation functions for industrial applications. Rob. Comput. Integr. Manuf., 83, 102559. doi: 10.1016/j.rcim.2023.102559* (https://www.sciencedirect.com/science/article/pii/S0736584523000352).

# Required modifications

The functions are very generic, but a few modifications are required in order to use the code with any robot.

* The name of the moveit\_commander variables must be:
    * **robot:** For the moveit\_commander.robot object
    * **scene:** For the moveit\_commander.planning\_scene\_interface object
    * **arm_right:** For the moveit\_commander.move\group object of the right arm.
    * **arm_left:** For the moveit\_commander.move\group object of the left arm.
    * **arms:** For the moveit\_commander.move\group object of the dual arm group.
    * **torso:** For the moveit\_commander.move\group object of the robot torso.

* The named target configurations must be modified to match the ones in the SRDF of your robot.

# How to launch the demo

All the required dependencies and pkgs must be installed, such as ROS Industrial, URDF, MoveIt...
First, launch the simulation environment (this has been modified and it is not exactly the same as can be found in the Motoman repository):
* If no physical robot is connected: roslaunch motoman\_sda10f\_moveit\_config demo\_no\_gripper.launch
* If the physical robot is connected: roslaunch motoman\_sda10f\_moveit\_config moveit\_planning\_execution\_no\_gripper.launch

Then, any of the graphs can be launched:
* Dual-arm EEF speed plot: roslaunch advanced\_manipulation\_pkg plot\_dual\_arm\_speeds.launch
* Left arm EEF speed plot: roslaunch advanced\_manipulation\_pkg plot\left\_arm\_speed.launch
* Left arm angular EEF speed plot: roslaunch advanced\_manipulation\_pkg plot\left\_arm\_angular\_speed.launch

Finally, launch the demo:
* roslaunch advanced\_manipulation\_pkg advanced\_manipulation.launch
