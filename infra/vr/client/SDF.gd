extends Node


onready var sdf = load("res://addons/sdf/main.gd").new()
onready var SDFActor = load("res://SDFActor.tscn")

puppet func spawn(name, config, tf):
  print("Spawn called, ", name, " at ", tf.origin)
  var inst = SDFActor.instance()
  inst.name = name
  inst.transform = tf
  var parsed = sdf.ParseAttrs(config)
  inst.add_child(parsed)
  inst.setup_controls()
  add_child(inst)
  return inst

puppetsync func remove(name):
  self.get_node(name).queue_free()
  print("Removed SDF %s" % name)
