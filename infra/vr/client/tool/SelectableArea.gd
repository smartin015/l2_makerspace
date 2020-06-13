extends Area

var selected = false
onready var mi = $MeshInstance

func _ready():
  mi.material_override = SpatialMaterial.new()
  _set_color()

func _set_color():
  var c = Color(0.5, 0.5, 0.5)
  if selected:
    c = Color(0, 0.5, 0)
  mi.material_override.albedo_color = c

func ui_raycast_hit_event(position, click, release):
  selected = not selected
  if click:
    print("HIT %s - %s" % [get_parent().name, selected])
  _set_color()
    
