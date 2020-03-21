extends Node

# TODO: Link this up to message type spec in infra/base/l2_msgs/msg/Object3D.msg
const TYPE_OBJ = 1
const TYPE_SDF = 2

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

  match args.object.type:
    TYPE_OBJ:
      ROSBridge.service_response("PushObject3D", id, {
        "success": false, 
        "message": "OBJ parsing not implemented"
      })
    TYPE_SDF:
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
