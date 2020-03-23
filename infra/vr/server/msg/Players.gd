# ROS service to accept new 3D objects and send them to the VR clients
# 
# Test with:
#  ros2 service call /l2/vr/PushObject3D l2_msgs/srv/PushObject3D '{object: {type: 1, name: 'test_obj', length: 0, data: ''}}'
extends Node

var polltmr
const TOPIC_TYPE = "l2_msgs/Object3DArray"
onready var sdf = get_node("/root/World/SDF")

func _ready():
  ROSBridge.topics.push_back(self)
  polltmr = Timer.new()
  polltmr.wait_time = 2.0
  polltmr.connect("timeout", self, "_poll") 
  add_child(polltmr)
  polltmr.start()

func advertisement(id):
  return { 
    "op": "advertise",
    "service": ROSBridge.NS + "/Object3D",
    "type": TOPIC_TYPE,
    "id": "%s_object3d" % id,
  }

func _poll():
  var objects = []
  
  for c in sdf.get_children():
    objects.append({
      name: c.name,
    })
  
  ROSBridge.publish("Object3D", TOPIC_TYPE, {
    objects: objects,
   }, "object3d")
