# Simple AddTwoInts service example for testing if everything's
# set up correctly.
# 
# Test with:
#   ros2 service call /l2/vr/AddTwoInts example_interfaces/srv/AddTwoInts '{a: 1, b: 2}'
extends Node

func _ready():
  ROSBridge.services.push_back(self)

func advertisement(id):
  return { 
    "op": "advertise_service",
    "type": "example_interfaces/AddTwoInts",
    "service": ROSBridge.NS + "/AddTwoInts",
    "id": "%s_addtwoints" % id,
  }

func maybe_handle(service, id, args):
  if service != ('%s/AddTwoInts' % ROSBridge.NS):
    return false
  
  ROSBridge.service_response("AddTwoInts", id, {
    "sum": args.a + args.b, 
  })
  return true
