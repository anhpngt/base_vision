**New updates on 24-Jun-2017: armor_detection node now has 2 modes

	mode = 0 for detection with both LEDs and circle stickers

	mode = 1 for detection with only circle stickers

Detection mode can be adjusted in the launch file, under the [mode] argument.


Adding package into workspace:


	cd ~/catkin_ws/src/

	git clone https://github.com/phamngtuananh/base_vision.git

	cd ~/catkin_ws/src/base_vision/cfg

	chmod a+x armor_color.cfg board_color.cfg building_blocks.cfg

	cd ~/catkin_ws

	catkin_make


Running the armor detection node:


	roslaunch base_vision armor_detection.launch (the default color of LEDs is red)

or

	roslaunch base_vision armor_detection.lanch color:=blue


Running the object detection for UAV: (detects circles)


	roslaunch base_vision cross.launch


Running the red ball targetting node:


	roslaunch base_vision target.launch

The output will be shown on screen, and also via topic /center, which is of type Vector3


To run the camera: make sure you have the usb_cam package in the workspace


To add the package into workspace:

	cd ~/catkin-ws/src

	git clone https://github.com/bosch-ros-pkg/usb_cam.git

	cd ..

	catkin_make


To run the camera

	roslaunch base_vision usb_cam.launch