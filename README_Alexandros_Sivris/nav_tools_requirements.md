## 1. Navigation tools package ##
### Requirements ###
As mentioned in the section [Visual localization](visual_localization.md)  the package requires the external packages:
1. usb_cam (https://github.com/bosch-ros-pkg/usb_cam.git)
2. hector_compressed_map_transport (optional, not recommended, https://github.com/tu-darmstadt-ros-pkg/hector_slam.git)
Further dependencies will be listed below

#### Dependencies ####
*	ROS Indigo running under Linux (14.04 LTS)
*	OpenCV 3 (developed with OpenCV 3.2.0-dev)
*	cv_bridge package of vision opencv project (if not already installed, it can be fetched from https://github.com/ros-perception/vision_opencv.git)
*	C/C++ compiler (development with gcc version 4.8.4)
*	RapidXML library (already included in the package include folder, no need to download)