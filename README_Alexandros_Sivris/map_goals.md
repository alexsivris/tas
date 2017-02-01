## 1. Navigation tools package ##
### Map goals ###
#### Files ####
*	navigation_tools/map_goals/src/main.cpp
*	navigation_tools/map_goals/src/XMLPoses.cpp
*	navigation_tools/map_goals/src/send_nav_goals.cpp
*	navigation_tools/map_goals/src/XMLPoses.hpp
*	navigation_tools/map_goals/src/send_nav_goals.h
*	navigation_tools/map_goals/src/poses.xml
#### Description ####
In this node I publish the waypoints for autonomous navigation and send them to the action client. The waypoints are loaded from an XML file and their origin is the `nav_origin` frame. 
#### How to run the node ####
Before running the map goals node, the `nav_origin` frame has to be broadcasted. Therefore, run the `visual_localization` node (`localization.launch`) *before* running the `map_goals` node. (`autonomous_driving.launch`). If the visual localization was not successful, set the argument `visual_localization_successful` in the `autonomous_driving.launch` file to **false** (default), otherwise to **true**. 
![](nav-origin-with-waypoints.png) 