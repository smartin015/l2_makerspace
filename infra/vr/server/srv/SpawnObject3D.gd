# ROS service to accept new 3D objects and send them to the VR clients
# 
# Test with:
#  ros2 service call /l2/vr/SpawnObject3D l2_msgs/srv/SpawnObject3D '{object: {type: 1, name: 'test_obj', length: 0, data: ''}}'
extends Node

# TODO: Link this up to message type spec in infra/base/l2_msgs/msg/Object3D.msg
enum type {OBJ = 1, SDF = 2, PROTO = 3}
const srvname = "SpawnObject3D"

onready var actors = get_node("/root/World/Actors")

func _ready():
  ROSBridge.services.push_back(self)

func advertisement(id):
  return { 
    "op": "advertise_service",
    "service": srvname,
    "type": "l2_msgs/SpawnObject3D",
    "id": "%s_spawnobject3d" % id,
  }

func maybe_handle(service, id, args, peer_id):
  print("%s vs %s" % [service, srvname])
  if service != srvname:
    return false

  var tf = Transform.IDENTITY
  if args.get("pose") != null:
    var pos = args.pose.get("position")
    if pos != null:
      tf.position = Vector3(pos.x, pos.y, pos.z)
    var quat = args.pose.get("orientation")
    if quat != null:
      tf.basis = Basis(Quat(quat.x, quat.y, quat.z, quat.w))

  tf = tf.scaled(Vector3.ONE * args.get("scale", 1.0))

  match int(args.object.type):
    type.OBJ:
      ROSBridge.service_response(srvname, id, {
        "success": false, 
        "message": "OBJ parsing not implemented"
      })
    type.SDF:
      # We forward the peer ID so that we can monitor the remote
      # simulated environment to autoremove the object
      actors.spawn(args.object.name, "SDF", args.object.data, tf, peer_id)
      ROSBridge.service_response(srvname, id, {
        "success": true, 
        "message": "SDF model %s created @ %s" % [args.object.name, tf],
      })
    type.PROTO:
      # We forward the peer ID so that we can monitor the remote
      # simulated environment to autoremove the object
      actors.spawn(args.object.name, "PROTO", args.object.data, tf, peer_id)
      ROSBridge.service_response(srvname, id, {
        "success": true, 
        "message": "PROTO model %s created @ %s" % [args.object.name, tf],
      })
    _:
      ROSBridge.service_response(srvname, id, {
        "success": false, 
        "message": "Unknown type %s" % args.object.type
      })
      
  return true
