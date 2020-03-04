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

puppetsync func remove(name):
  self.get_node(name).queue_free()
  print("SDF %s removed" % name)
  
