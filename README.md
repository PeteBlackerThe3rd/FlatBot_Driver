FlatBot_Driver & support packages
=================================

This repository is dependant upon several ROS packages beyond the standard desktop-full installation. The following extra pacakges need to be installed for it to operate correctly.

* ros-<version>-moveit
* ros-<version>-ros-control
* ros-<version>-ros-controllers

Packages
--------

### Flat_Bot_Driver

Moveit Compatible ROS driver for our 2D table robot Flat Stanley. This node is essentially a moveit compatible driver for any set of herkulex servos connected via UART. The current version is designed to run on a Rpi3 using the GPIO UART connection.

### Assembly_Control

The Assembly_Control package containing nodes to manage a set of nodes, and plan & execute pick and place operations on a real or simulated setup.

### Flat_Bot_Support

Moveit_Support package containing moveit & gazebo compatible URDF definition of the robot and associated 3D models

### Flat_Bot_Moveit

Collection of launch files and YAML files, originally created using the moveit setup assistant. These files are used to start the moveit motion planning sub-system. 

### Flat_Bot_Msgs

A message only package defining the types used to describe the module communication and interface of the assembly_control node