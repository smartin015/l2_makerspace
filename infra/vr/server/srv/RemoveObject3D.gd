# ROS service to accept new 3D objects and send them to the VR clients
# 
# Test with:
#  ros2 service call /l2/vr/RemoveObject3D std_msgs/String '{object: {type: 1, name: 'test_obj', length: 0, data: ''}}'
extends Node

onready var sdf = get_node("/root/World/SDF")

func _ready():
  ROSBridge.services.push_back(self)

func advertisement(id):
  return { 
    "op": "advertise_service",
    "service": ROSBridge.NS + "/RemoveObject3D",
    "type": "l2_msgs/RemoveObject3D",
    "id": "%s_rmobject3d" % id,
  }

func maybe_handle(service, id, args):
  if service != ('%s/RemoveObject3D' % ROSBridge.NS):
    return false

  var errstr = sdf.remove(args.name)
  ROSBridge.service_response("RemoveObject3D", id, {
    "success": errstr != null, 
    "message": errstr,
  })
  return true
