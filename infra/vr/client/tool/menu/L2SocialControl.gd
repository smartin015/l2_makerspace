extends ReferenceRect

onready var players = $VBoxContainer/ScrollContainer/Players

signal social_action

func _ready():
  for p in gamestate.players.get_children():
    var plabel = Label.new()
    plabel.text = p.alias
    players.add_child(plabel)

func _on_Edit_pressed():
 emit_signal("social_action", "", "edit")
