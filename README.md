# ROS Package: HTC Vive Tracker Localization
by Adam Pollack

This branch uses the Vive Tracker Localization package from the master branch of this repository, but converts it to use ROS to publish the pose of the Vive Tracker.  

The goal of this project is the use HTC Lighthouses to determine the location of an HTC Vive Tracker using only a Raspberry Pi and software. The final executable prints the pose of the tracker relative to one lighthouse.

This package uses code from 'libsurvive' to interface with the HTC Vive Tracker. More info here: https://github.com/cnlohr/libsurvive

## Installation

1. Install dependencies (libUSB, libX11, zlib, OpenCV)  
	a. libUSB: sudo apt-get install libusb-1.0-0-dev  
	b. libX11: sudo apt-get install libx11-dev  
	c. OpenCV: sudo apt-get install libopencv-dev  

Clone vive_tracker_loc into a ROS workspace

Install udev rules (this is needed to modify permissions for the Vive Tracker):  
`cp vive_tracker_loc/81-vive.rules /etc/udev/rules.d/` and rebooting  

cd ros_ws  
catkin_make  

To publish pose data: `rosrun vive_tracker_loc vive_pose_node`  

### Nodes  
`vive_pose_node`: Broadcasts the transform from lighthouse to tracker to /tf topic. Publishes raw IMU data to /tracker_imu.  

<!-- `tf_calibration`: Determines the transform from the base of Sawyer to the lighthouse using

### Launch Files  
`calibrate.launch`: Launches  -->

## IMPORTANT  

Do NOT kill runposer using C-\\, this will render the Vive Tracker useless until pairing again with SteamVR
