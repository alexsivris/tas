## 1. Navigation tools package ##
### Waypoint setter ###
#### Files ####
*	navigation_tools/waypoint_setter/main.cpp
*	navigation_tools/waypoint_setter/src/waypointsetter.cpp
*	navigation_tools/waypoint_setter/src/waypointsetter.h

#### Description ####
This node is a tool for setting the waypoint poses for the `map_goals` node, i.e. preparing the XML file. Unfortunately I could not finish this node, but the idea is that it subscribes to the `/initialpose` topic and reads all the poses that are choses in Rviz with the `2D Pose Estimate` tool. In the end all of the waypoints are saved in an XML file that can be exported to the `map_goals` node.

#### How to run the node ####
This node cannot be run, because it isn't finished yet.