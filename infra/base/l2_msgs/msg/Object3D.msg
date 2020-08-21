string PG_COLS="-origin -pose -scale"

int64 id

uint8 TYPE_UNKNOWN = 0
uint8 TYPE_OBJ = 1

# Gazebo SDF format
uint8 TYPE_SDF = 2

# Webots PROTO format
uint8 TYPE_PROTO = 3

# L2 Supervisor node format
uint8 TYPE_SUPERVISOR = 4
uint8 type

string name
string data

geometry_msgs/Vector3 origin
geometry_msgs/Pose pose
geometry_msgs/Vector3 scale
