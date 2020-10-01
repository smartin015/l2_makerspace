# A step can be any operation performend on one or more Parts,
# e.g. "add this Part", "cut this Part", "glue these two Parts together"
l2_msgs_sim/Part[] parts

uint8 OP_TYPE_INVALID = 0
uint8 OP_TYPE_ADD = 1
uint8 op_type

l2_msgs_sim/OpAdd op_add
