string PG_COLS="-items owner:int64"

int64 id 
string name

# id must be defined; all else optional
l2_msgs/L2Actor owner 

l2_msgs/ProjectItem[] items

# owner is actively working on project right now
bool active
