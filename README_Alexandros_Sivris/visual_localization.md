## 1. Navigation tools package ##
### Visual Localization ###
#### Files ####
*	navigation_tools/visual_localization/main.cpp
*	navigation_tools/visual_localization/src/visuallocalization.h
*	navigation_tools/visual_localization/src/visuallocalization.cpp
*	navigation_tools/visual_localization/src/landmarkdetector.h
*	navigation_tools/visual_localization/src/landmarkdetector.cpp
*	navigation_tools/visual_localization/src/landmarkmatcher.h
*	navigation_tools/visual_localization/src/landmarkmatcher.cpp
*	navigation_tools/visual_localization/src/my_types.h
#### Description ####
In this node I calculate the pose estimate of the car using a non-linear system of three equations. These equations result from the circles drawn around the three detected landmarks with radius equal to the distance between the car and the corresponding landmark. This distance is read out from the laser scanner
#### How to run the node ####
To run the node I prepared a launch file called `localization.launch` which is located in the "navigation_tools/launch" folder. The recommended way to run the node is by loading the map from an image file (which is set by default in line 5 of `localization.launch`). Alternatively, the map could also be fetched from the node `map_to_image_node` from the `ector_compressed_map_transport` package (this is not a good option though, because the map is very large and this would require that all the data is published on a ROS topic). 
The node will start by finding the position of the loaded landmarks in the map and then it will wait for image data coming from the usb camera, which can be mounted on top of the car. Therefore, the package `usb_cam`  **must** be launched in order to receive image data in the topic "/usb_cam/image_raw".