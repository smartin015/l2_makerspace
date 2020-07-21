extends ReferenceRect

signal clear
signal set_topic

export(String) var text setget _set_text, _get_text
export(PoolStringArray) var topics setget _set_topics
 
onready var label = $MarginContainer/VBoxContainer/RichTextLabel;
onready var topic = $MarginContainer/VBoxContainer/HBoxContainer/Topic

func _set_text(s):
  label.text = s
  
func _get_text():
  return label.text

remotesync func clear():
  label.text = ""

func _on_Clear_pressed():
  emit_signal("clear")

func set_active_topic(t: String):
  topic.text = "Topic: " + t

func _set_topics(ts):
  print("Setting topics to %s" % ts)
  topics = ts
  var popup = topic.get_popup()
  popup.clear()
  for v in range(len(topics)):
    popup.add_item(topics[v], v)
  #popup.set_position(Vector2(100,100))
  popup.connect("id_pressed", self, "_popupMenuChoice")
  popup.show()

func _popupMenuChoice(id):
  emit_signal("set_topic", topics[id])
