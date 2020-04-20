extends Node


onready var sdfParser = load("res://addons/parse/sdf.gd").new()
onready var protoParser = load("res://addons/parse/proto.gd").new()
onready var Actor = load("res://actor/Actor.tscn")

puppet func spawn(name, objtype, config, tf, peer_id):
  print("%s Actor %s @ %s from ROS(%s)" % [objtype, name, tf.origin, peer_id])
  var inst = Actor.instance()
  inst.name = name
  inst.transform = tf
  match objtype:
    "SDF":
      inst.add_child(sdfParser.ParseAttrs(config))
    "PROTO":
      inst.add_child(protoParser.ParseAttrs(config))
    _:
      print("ERROR: No parser for objtype " + objtype)
      return
  
  inst.setup_controls()
  add_child(inst)
  return inst

puppetsync func remove(name):
  self.get_node(name).queue_free()
  print("Actor %s removed" % name)
