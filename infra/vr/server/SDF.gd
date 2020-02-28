extends Node

onready var SDFActor = load("res://SDFActor.tscn")

puppetsync func spawn(name, sdf, tf):
  var inst = SDFActor.instance()
  inst.name = name
  inst.transform = tf
  inst.sdf = sdf
  
  self.add_child(inst)
  print("Initialized SDF %s tf %s" % [name, tf])
  return inst

puppetsync func remove(name):
  self.get_node(name).queue_free()
  print("Removed SDF %s" % name)
  
