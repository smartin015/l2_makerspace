string name
diagnostic_msgs/KeyValue[] params

builtin_interfaces/Time started
builtin_interfaces/Time stopped
string[] container_ids

# Status and code are set to "success", 0 if all containers 
# complete successfully.
# Otherwise it's set to one of the container's stderr
# and code.
string status
int32 code
