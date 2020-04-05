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
  var names = []
  for c in get_children():
    if c.name == name:
      c.queue_free()
      print("SDF %s removed" % name)
      return null
    names.append(c.name)
  return "Node not found; candidates %s" % names 
  
