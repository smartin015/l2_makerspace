extends Node

onready var PuppetPlayer = load("res://PuppetPlayer.tscn")

puppet func spawn(id):
  if str(id) == gamestate.player.name:
    print("GDT(%s) server ack; init complete" % id)
    gamestate.is_initialized = true
    return

  if gamestate.players.find_node(str(id)):
    print("GDT(%s) already spawned" % id)
    return

  var inst = PuppetPlayer.instance()
  # node name & network master must be the same across all peer
  inst.name = str(id) 
  inst.set_network_master(id)
  add_child(inst)
  print("GDT(%s) spawned" % id)

func clear():
  for p in get_children():
    remove(p.name)

puppet func remove(id):
  print("REMOVE %s" % id)
  var p = get_node(str(id))
  if p:
    p.queue_free()
    print("GDT(%s) removed" % id)
