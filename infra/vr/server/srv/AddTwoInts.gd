# Simple AddTwoInts service example for testing if everything's
# set up correctly.
# 
# Test with:
#   ros2 service call /l2/vr/AddTwoInts example_interfaces/srv/AddTwoInts '{a: 1, b: 2}'
extends Node

const srvname = "AddTwoInts"

func _ready():
  #ROSBridge.services.push_back(self)
  pass

func advertisement(id):
  return { 
    "op": "advertise_service",
    "type": "example_interfaces/srv/AddTwoInts",
    "service": srvname,
    "id": "%s_addtwoints" % id,
  }

func maybe_handle(service, id, args, peer_id):
  if service != srvname:
    return false
  
  ROSBridge.service_response(srvname, id, {
    "sum": args.a + args.b, 
  })
  return true
