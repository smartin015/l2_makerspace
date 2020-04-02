extends Node


onready var parser = load("res://addons/sdf/main.gd").new()
onready var SDFActor = load("res://sdf/SDFActor.tscn")

puppet func spawn(name, sdf, tf, peer_id):
  print("SDF spawn %s @ %s from ROS(%s)" % [name, tf.origin, peer_id])
  var inst = SDFActor.instance()
  inst.name = name
  inst.transform = tf
  var parsed = parser.ParseAttrs(sdf)
  inst.add_child(parsed)
  inst.setup_controls()
  add_child(inst)
  return inst

puppetsync func remove(name):
  self.get_node(name).queue_free()
  print("SDF remove %s" % name)
