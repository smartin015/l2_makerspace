# ROS topic listing current lock state
# 
# Test with:
#  ros2 topic echo /l2/vr/Locks
extends Node

var polltmr
const TOPIC_TYPE = "l2_msgs/msg/VRLockStatus"

func _ready():
  ROSBridge.topics.push_back(self)
  polltmr = Timer.new()
  polltmr.wait_time = 10.0
  polltmr.connect("timeout", self, "_poll") 
  add_child(polltmr)
  polltmr.start()

func advertisement(id):
  return { 
    "op": "advertise",
    "topic": "Locks",
    "type": TOPIC_TYPE,
    "id": "%s_locks" % id,
  }

func _poll():
  var ls = []
  var l = locks.locks
  for k in l:
    if l[k] != null:
      ls.append({
        "key": str(k),
        "value": str(l[k]),
      })
  
  ROSBridge.publish("Locks", TOPIC_TYPE, {
    "locks": ls,
  }, "locks")
