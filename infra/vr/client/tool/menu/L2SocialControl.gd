extends ReferenceRect

onready var players = $VBoxContainer/ScrollContainer/Players
onready var SocialItem = load("res://tool/menu/SocialItem.tscn")

signal social_action

func _ready():
  for p in gamestate.players.get_children():
    var si = SocialItem.instance()
    si.find_node("color", true, false).color = p.color
    si.find_node("name", true, false).text = p.alias
    si.get_node("workspace").text = p.ws
    players.add_child(si)

func _on_Edit_pressed():
 emit_signal("social_action", "", "edit")
