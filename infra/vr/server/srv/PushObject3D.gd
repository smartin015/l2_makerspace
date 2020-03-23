# ROS service to accept new 3D objects and send them to the VR clients
# 
# Test with:
#  ros2 service call /l2/vr/PushObject3D l2_msgs/srv/PushObject3D '{object: {type: 1, name: 'test_obj', length: 0, data: ''}}'
extends Node

# TODO: Link this up to message type spec in infra/base/l2_msgs/msg/Object3D.msg
enum type {OBJ = 1, SDF = 2}

onready var sdf = get_node("/root/World/SDF")

func _ready():
  ROSBridge.services.push_back(self)

func advertisement(id):
  return { 
    "op": "advertise_service",
    "service": ROSBridge.NS + "/PushObject3D",
    "type": "l2_msgs/PushObject3D",
    "id": "%s_pushobject3d" % id,
  }

func maybe_handle(service, id, args):
  if service != ('%s/PushObject3D' % ROSBridge.NS):
    return false

  match int(args.object.type):
    type.OBJ:
      ROSBridge.service_response("PushObject3D", id, {
        "success": false, 
        "message": "OBJ parsing not implemented"
      })
    type.SDF:
      sdf.spawn(args.object.name, args.object.data, Transform.IDENTITY)
      ROSBridge.service_response("PushObject3D", id, {
        "success": true, 
        "message": "SDF model %s created" % args.object.name
      })
    _:
      ROSBridge.service_response("PushObject3D", id, {
        "success": false, 
        "message": "Unknown type %s" % args.object.type
      })
      
  return true
