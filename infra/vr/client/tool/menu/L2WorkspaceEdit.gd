extends ReferenceRect

var orig
var name_edit
signal close_edit

func _ready():
  name_edit = find_node("NameEdit", true, false)

func get_form():
  return {
    "name": name_edit.text,
  }

func set_form(f):
  for k in f:
    match k:
      "name":
        name_edit.text = f[k]
      _:
        print("unknown form element %s" % k)

func fill(form):
  set_form(form)
  orig = get_form()

func _on_Done_pressed():
  emit_signal("close_edit", orig, get_form())
