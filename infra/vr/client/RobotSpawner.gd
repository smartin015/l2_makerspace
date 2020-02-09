extends Spatial

var sdf = load("res://addons/sdf/main.gd").new()

# Called when the node enters the scene tree for the first time.
func _ready():
  var f = File.new()
  # Load example control zone
  f.open("res://test/unit/control_zone.sdf", File.READ)
  var t = f.get_as_text()
  f.close()
  var models = sdf.ParseAttrs(t)
  for m in models.values():
    self.add_child(m)
    print("Added SDF model with ", m.get_children()[0].get_children()[0])
