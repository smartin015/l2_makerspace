# ROS topic to publish current 3D objects
# 
# Test with:
#  ros2 topic echo /l2/vr/Object3D
extends Node

var polltmr
const TOPIC_TYPE = "l2_msgs/Object3DArray"
onready var sdf = get_node("/root/World/SDF")

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
    "topic": ROSBridge.NS + "/Object3D",
    "type": TOPIC_TYPE,
    "id": "%s_object3d" % id,
  }

func _poll():
  var objects = []
  
  for c in sdf.get_children():
    objects.append({
      "name": c.name,
    })
  print(JSON.print(objects))
  ROSBridge.publish("Object3D", TOPIC_TYPE, {
    "objects": objects,
   }, "object3d")
