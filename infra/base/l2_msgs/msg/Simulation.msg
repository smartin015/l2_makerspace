# Namespace is prepended to published messages for this simulation
string namespace

# Path the worker reports this message to
string report_path

# worker_id is the ID of the ros node controlling the simulation.
# When this node is terminated, dependent nodes will also clean up and terminate.
string worker_id

l2_msgs/Object3D object
