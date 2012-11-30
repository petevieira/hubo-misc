hubo-misc
=========
This repo is for Hubo related work not for distribution.


hubo-gui
--------
This directory is for the wxPython GUI for Hubo

kinematics
----------
This directory is currently for matlab version of kinematics and dynamics

### arm.m

This file contains the inverse kinematics for a 2-DOF manipulator

urdf
----
This directory is for Hubo's URDF files for DART/GRIP

### hubo.xml

This file contains the xml code for Hubo's entire body.

Initialze ROS URDF Parser:
	$ rosmake urdf_parser

Run URDF Parser to check URDF file
	$ rosrun urdf_parser check_urdf hubo.xml

hubo-documentation
------------------
This directory is for useful documentation on Hubo

robotics-textbooks
------------------
This directory contains recommended robotics textbooks

robotics-papers
---------------
This directory contains possibly useful robotics papers
