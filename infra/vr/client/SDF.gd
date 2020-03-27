extends Node


onready var sdf = load("res://addons/sdf/main.gd").new()
onready var SDFActor = load("res://SDFActor.tscn")

puppet func spawn(name, config, tf, peer_id):
  print("SDF spawn %s @ %v from ROS(%s)" % [name, tf.origin, peer_id])
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
  print("SDF remove %s" % name)
