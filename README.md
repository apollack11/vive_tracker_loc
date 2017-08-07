## README for HTC Vive Localization Project
by Adam Pollack

The goal of this project is the use HTC Lighthouses to determine the location of an HTC Vive Tracker using only a Raspberry Pi and software

I found a package called 'libsurvive' which I will be building upon for this project. More info here: https://github.com/cnlohr/libsurvive


## INSTRUCTIONS FOR SETUP  

0. Make sure git is installed
1. Install dependencies (libUSB, libX11, zlib, OpenCV)  
	a. libUSB: sudo apt-get install libusb-1.0-0-dev  
	b. libX11: sudo apt-get install libx11-dev  
	c. OpenCV: sudo apt-get install libopencv-dev  
2. Install library from GitHub repo (master branch): git clone https://github.com/apollack11/libsurvive.git


## INSTRUCTIONS FOR USE  

cd adam_libsurvive  
make  
Install udev rules by: cp setup/81-vive.rules to /etc/udev/rules.d/ and rebooting  
Edit config.json according to sample-config.json in "setup/" directory  
./runposer  


## IMPORTANT  

Do NOT kill runposer using C-\, this will render the Vive Tracker useless until pairing again with SteamVR