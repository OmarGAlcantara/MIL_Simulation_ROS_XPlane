# MIL_Simulation_ROS_XPlane

This repo represents a MIL Simulation in the XPlane 11 Flight Simulation Environment using XPlaneROS and XPlaneConnect.


Dependencies:

-XPlane11

-Ubuntu 18.04

-ROS Melodic

-XplaneROS

-XPlaneConnect


Assuming Ubuntu 18, XPlane 11 and ROS Melodic full desktop version are installed.


XPLANE SET UP

Download from this repository the XPlaneConnect folder and paste it in the XPlane plugins route. If XPlane was purchased from Steam the route would be "/home/youruser/.local/share/Steam/steamapps/common/X-Plane 11/Resources/plugins".

Download the My_custom_aircraft folder from this repo and paste it in XPlane/Aircraft route.

Download the e63 and clark-y airfoil files from the Airfoils folder in this repo and paste them in the XPlane/Airfoils route.


Double check the XPlaneConnect plugin is working by launching an XPlane simulation and clicking on the plugins tab.


Inside the Setting - Data Output tab on XPlane fully increase the UDP Rate and activate the Send network data output option. In the UDP adress type "127.0.0.1" and in Port "49005".



COMMUNICATION SETUP


Clone the XPlaneConnect repository by typing in a terminal $git clone https://github.com/nasa/XPlaneConnect.git


If not already done so, create a catkin workspace with catkin tools:


$sudo apt-get install python-catkin-tools

$mkdir -p ~/xplane_ros_ws/src

$cd ~/xplane_ros_ws

$catkin init


After that we will install our fork of ROSplane. Download the src folder from the following url "https://cinvestav365-my.sharepoint.com/:f:/g/personal/omar_garcia_cinvestav_mx/EgPVlCLWZzRJh9adac1V2fABYqtnc4FqSADwBfdWTE3kDQ?e=coI14Y" and paste it inside the xplane_ros_ws folder.


In a terminal type:

	$cd rosflight/
 
	$git submodule update --init --recursive
 
  	$sudo apt-get install ros-melodic-eigen-stl-containers
  
 	$cd ..
  
  	$catkin build
  
  	$source ~/xplane_ros_ws/devel/setup.bash
  

For convenience, you can add the source ~/xplane_ros_ws/devel/setup.bash statement in your ~/.bashrc file.


Move to your catkin workspace:

	$cd ~/xplane_ros_ws/src


Install system dependencies:

	$sudo apt-get install python-wstool python-catkin-tools

Source and compile:

	$catkin build

	$source ~/xplane_ros_ws/devel/setup.bash

Testing if XPlaneROS works:

Open X-Plane 11 (X-Plane 11/X-Plane_x86_64).

Select New Flight.

In the LOCATION panel, set the location to Butler airport (ID : KBTP) with Runway 26.

Click Start Flight.

You can then start XPlaneROS with


	$roslaunch xplane_ros default.launch

The default.launch provides the bare-minimum structure in order to interface with XPlane. The xplane_ros_wrapper node will provide the odometry data from XPlane and it will listen to Xplane commands from the user application on the topic /xplane/my_control and then send them to XPlane.


The default.launch file also has a launch command for the rviz visualization. If rostopics are shown when inputoong $rostopic list that means XplaneROS is working.


SETTING UP A SIMULATION

For the simulation create a ROS package and paste inside the folder scripts and the folder launch.

Make sure to build your workspace

  	$cd ..
  
 	$catkin build
  
  	$source ~/xplane_ros_ws/devel/setup.bash
  

to run a simulation open a simulation on xplane selecting the aircraft intelaerortf2 and the location to Butler airport (ID : KBTP) with Runway 26. Launch the file to start the simulation


$file-route/python3 Control_QUAD_ROS.py



