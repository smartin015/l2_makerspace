extends Node

onready var Canvas = load("res://panel/Canvas.tscn")
onready var Chart = load("res://panel/Chart.tscn")
onready var Console = load("res://panel/Console.tscn")
onready var Screen = load("res://panel/Screen.tscn")
onready var Sequence = load("res://panel/sequence/Sequence.tscn")

onready var toolmap = {
  "CANVAS": Canvas,
  "CHART": Chart,
  "CONSOLE": Console,
  "SCREEN": Screen,
  "SEQUENCE": Sequence,
}

remote func spawn(name, objtype, tf, ws):
  print("%s %s @%s ws %s" % [objtype, name, tf.origin, ws])
  var toolscene = toolmap.get(objtype)
  if toolscene == null:
    print("ERROR: unknown tool objtype " + objtype)
    return
    
  var inst = toolscene.instance()
  inst.name = name
  inst.transform = tf
  inst.ws = ws
  add_child(inst)
  
  # Only spawn for players in the same workspace
  for p in gamestate.players.get_children():
    if p.ws == ws:
      rpc_id(p.get_network_master(), "spawn", name, objtype, tf)
  return inst
