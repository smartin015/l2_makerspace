extends Node

onready var SDFActor = load("res://sdf/SDFActor.tscn")

puppetsync func spawn(name, sdf, tf, peer_id):
  var inst = SDFActor.instance()
  inst.name = name
  inst.transform = tf
  inst.sdf = sdf
  inst.peer_id = peer_id
  
  add_child(inst)
  print("SDF %s spawned (%d total)" % [name, len(get_children())])
  return inst

# Returns null if no error, error string otherwise
puppetsync func remove(name):
  var n = get_node(name)
  if n == null:
    return "Node not found"
  n.queue_free()
  print("SDF %s removed" % name)
  return null
  
