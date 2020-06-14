extends ReferenceRect

onready var players = $VBoxContainer/ScrollContainer/Players

func _ready():
  for p in gamestate.players.get_children():
    var plabel = Label.new()
    plabel.text = p.name
    players.add_child(plabel)
