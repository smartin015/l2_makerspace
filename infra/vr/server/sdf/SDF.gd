extends Node

onready var SDFActor = load("res://SDFActor.tscn")

puppetsync func spawn(name, sdf, tf):
  var inst = SDFActor.instance()
  inst.name = name
  inst.transform = tf
  inst.sdf = sdf
  
  self.add_child(inst)
  print("SDF %s spawned" % name)
  return inst

# Returns null if no error, error string otherwise
puppetsync func remove(name):
  var n = self.get_node(name)
  if n == null:
    return "Node not found"
  n.queue_free()
  print("SDF %s removed" % name)
  return null
  
