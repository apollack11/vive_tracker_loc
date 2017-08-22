# HTC Vive Localization Project
by Adam Pollack

The goal of this project is the use HTC Lighthouses to determine the location of an HTC Vive Tracker using only a Raspberry Pi and software. The final executable prints the pose of the tracker relative to one lighthouse.

This package uses code from 'libsurvive' to interface with the HTC Vive Tracker. More info here: https://github.com/cnlohr/libsurvive


## INSTRUCTIONS FOR SETUP  

0. Make sure git is installed
1. Install dependencies (libUSB, libX11, zlib, OpenCV)  
	a. libUSB: sudo apt-get install libusb-1.0-0-dev  
	b. libX11: sudo apt-get install libx11-dev  
	c. OpenCV: sudo apt-get install libopencv-dev  
2. Install library from GitHub repo (master branch): git clone https://github.com/apollack11/vive_tracker_loc.git


## INSTRUCTIONS FOR USE  

Install udev rules (this is needed to modify permissions for the Vive Tracker):  
`cp vive_tracker_loc/81-vive.rules /etc/udev/rules.d/` and rebooting  

cd vive_tracker_loc  
make  
./runposer  

## IMPORTANT  

Do NOT kill runposer using C-\\, this will render the Vive Tracker useless until pairing again with SteamVR
