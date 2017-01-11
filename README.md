# Babybot

Automatic cradle for helping handicapped parents to take care of their newborn.

Babybot components: 
	- body possessing omni wheels giving it a holonomous behavior
	- webcam, for recognizing the parent thanks to a green sphere held by their wheelchair
	- Lidar (2D laser sensor)
	- Kinect (3D camera)
	- embedded computer 
	- Arduino card for the communication between every component

Language: C++

Used tools: ROS, PCL library, KDevelop

code_Arduino: Arduino code

code_ROS: ROS_code
  - babybot: containing the global launch file
  - babybot_freenect: launching the Kinect and filtering its data
  - babybot_navigation: trajectory planning
  - camera_usb: detection of the parent through the webcam
  - changement_repere: connections between the tf frames
  - comanip: comanipulation mode
  - ira_laser_tools_master: merging of the 2D-signals from the Lidar and the Kinect
  - lidar: launching the Lidar and filtering its data
  - manette: launching the controller and treating its data
  - rosserial: communicating with the Arduino card
  - rplidar_ros: launching the Lidar (used by "lidar" package)
  - usb_cam: launching the webcam (used by "camera_usb" package)
  
  Launching all the code : babybot/launch/babybot.launch
