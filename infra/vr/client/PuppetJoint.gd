# This is different than the typical godot Joint classes, which
# actually attempt to solve joint positions. Here we're simply
# driving the joint values from elsewhere, dictated by ROS
extends Spatial

const REVOLUTE = 1
const PRISMATIC = 2

var type: int
# var limits: Vector2
var axis: Vector3
# var child: Spatial
# Initial child position
var origin: Transform

func _ready():
  origin = transform
  
func apply(val: float):
  match type:
    REVOLUTE:
      transform = origin.rotated(axis, val)
    PRISMATIC:
      transform = origin.translated(val * axis)
    _:
      print("Unsupported joint type ", type)

func toString():
  return "type: %s axis: %s origin: %s" % [type, axis, origin]
