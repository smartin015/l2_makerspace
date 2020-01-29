extends Spatial

var sdf = load("res://addons/sdf/main.gd").new()

# Called when the node enters the scene tree for the first time.
func _ready():
  var f = File.new()
  f.open("res://test/unit/basic.sdf", File.READ)
  var t = f.get_as_text()
  f.close()
  var models = sdf.ParseAttrs(t)
  for m in models.values():
    self.add_child(m)
