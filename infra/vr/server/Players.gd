extends Node

onready var PuppetPlayer = load("res://PuppetPlayer.tscn")

puppetsync func spawn(id, origin, ws):
  var p = PuppetPlayer.instance()
  p.name = str(id) # Important
  p.set_network_master(id) # Important
  p.ws = ws
  var t = Transform.IDENTITY
  t.origin = origin
  p.puppet_transform = t
  p.transform.origin = origin
  add_child(p)
  print("GDT(%s) spawned" % id)

puppetsync func remove(id):
  var p = get_node(str(id))
  if p:
    p.queue_free()
    print("GDT(%s) removed" % id)
