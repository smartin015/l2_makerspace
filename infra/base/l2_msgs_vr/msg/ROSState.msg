# This describes the interactive bits of 
# ROS2, useful to synchronize state of ROS with
# external systems (e.g. a Godot server connected with
# ros2-web-bridge)
# Each pair of arrays are in lock-step, e.g.
# topic_types[i] contains the type of topics[i]

string[] topics
string[] topic_types

string[] services
string[] service_types

string[] nodes
string[] node_namespaces
