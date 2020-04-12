extends Node

onready var Actor = load("res://actor/Actor.tscn")

puppetsync func spawn(name, objtype, config, tf, peer_id):
  var inst = Actor.instance()
  inst.name = name
  inst.transform = tf
  inst.objtype = objtype
  inst.config = config
  inst.peer_id = peer_id
  add_child(inst)
  print("%s Actor %s spawned (%d total)" % [objtype, name, len(get_children())])
  return inst

# Returns null if no error, error string otherwise
puppetsync func remove(name):
  var names = []
  for c in get_children():
    if c.name == name:
      c.queue_free()
      print("Actor %s removed" % name)
      return null
    names.append(c.name)
  return "Node not found; candidates %s" % names 
  
