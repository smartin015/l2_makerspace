string PG_COLS="roles:int64"

# Actors are autonomous entities in the L2 environment. They can be humans or
# robots, but they have a unified "type" so that access control and other
# special handling can be done without having to consider them separately. This
# makes it easier for robots and automation to replace manual "human" efforts
# over time.
int64 id
string name
bool human

# === Optional fields ===

# Actors can play a variety of roles (e.g. reading from storage etc)
l2_msgs/L2Role[] roles
