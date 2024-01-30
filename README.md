# Robosimgen (robot simulation generator) demo

A user-friendly tool that streamlines the establishment of a minimal working example of a robot simulation in Gazebo. Robot parameters are adjusted through rqt_reconfigure and the preview of the output can be seen in RVIZ through published markers. Once satisfied, standardized and ROS-compatible packages are generated that can be se seamleasly incoporated into a ROS workspace. Additional changes can later be done in the generated URDFs.

# How to run
Clone package into src/ of ros workspace, compile and source, then

`roslaunch robosimgen robosimgen.launch`

set the robot parameters through rqt and see the live changes in rviz. After your are happy, run the generated gazebo model with

(if this is the first time generating the model first recompile and source the workspace, after that recompiling is not needed)

`roslaunch robosim_gazebo empty_world.launch`




## Vision: 
Currently only a very basic differential drive robot is supported as a demo of the concept and later other common robots (e.g., differential drive, Ackermann, simple arms) can be added.

While the demo focuses on basic design adjustments, future iterations could include adding sensors (e.g., lidars, cameras, buttons) or even 3D renders of the robot. The exported robot would already include implemented links, joints, controllers, and macros, making it readily executable.

The UI could let you choose among the implemented robot models and show the relevant parameters needed for generating a model. The complexity of generated collisions, joints, added sensors and their params, ... is all a matter of implementation.

This should all be ported to ROS2 and Gazebo (former Ignition) eventually. Now its in Noetic and Gazebo Classic.

## Conversation

Innitial conversation about this: https://discourse.ros.org/t/project-proposal-visual-gazebo-robot-model-generator/35658