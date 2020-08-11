extends Spatial

const GRAB_TYPE="3dtransformer"
var control_zone # String passed in to find `zone`
var pos_topic
onready var xbod = $X
onready var ybod = $Y
onready var zbod = $Z
var ui

func _ready():
  ui = find_node("3DTransformUI", true, false)
  if ui == null:
    print("ERROR: Could not find 3DTransformUI for 3DTransformer")

func oq_can_static_grab(_b, _grab_area, _controller, _overlapping_bodies):
  return true
  
const MOVE_THRESHOLD_SQUARED = 0.01*0.01
var debounce = 1.0
var lastpos = Vector3.ZERO

func handle_grab(obj, delta_pos):
  # TODO delta from *selected* pos
  xbod.scale.x = 1
  ybod.scale.y = 1
  zbod.scale.z = 1
  match obj:
    xbod:
      xbod.scale.x = 1.5
      ui.x = delta_pos.x
    ybod:
      ybod.scale.y = 1.5
      ui.y = delta_pos.y
    zbod:
      zbod.scale.z = 1.5
      ui.z = delta_pos.z
    _:
      print("Unknown grabbed body %s" % obj)
