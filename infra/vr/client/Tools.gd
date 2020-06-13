extends Node

onready var Canvas = load("res://tool/canvas/Canvas.tscn")
onready var Chart = load("res://tool/chart/Chart.tscn")
onready var Console = load("res://tool/console/Console.tscn")
onready var Screen = load("res://tool/screen/Screen.tscn")
onready var Sequence = load("res://tool/sequence/Sequence.tscn")
onready var L2Control = load("res://tool/menu/L2Control.tscn")

puppet func spawn(name, objtype, tf, local=false):
  print("%s %s @%s" % [objtype, name, tf.origin])
  var inst
  match objtype:
    "CANVAS":
      inst = Canvas.instance()
    "CHART":
      inst = Chart.instance()
    "CONSOLE":
      inst = Console.instance()
    "SCREEN":
      inst = Screen.instance()
    "SEQUENCE":
      inst = Sequence.instance()
    "MENU":
      inst = L2Control.instance()
    _:
      print("ERROR: unknown tool objtype " + objtype)
      return
  inst.name = name
  inst.transform = tf
  add_child(inst)
  
  # Indicate to the server's instance of this tool
  # that we're ready to receive initial data/config.
  if !local:
    inst.rpc_id(1, "setup_request")
  return inst

puppetsync func remove(name):
  self.get_node(name).queue_free()
  print("Actor %s removed" % name)
