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
var vel: float = 0

func _ready():
  origin = transform
  
func apply(pos, vel: float):
  self.vel = vel
  match type:
    REVOLUTE:
      transform = origin.rotated(axis, pos)
    PRISMATIC:
      transform = origin.translated(pos * axis)
    _:
      print("Unsupported joint type ", type)

func _process(delta):
  if axis == Vector3.ZERO:
    return
  match type:
    REVOLUTE:
      transform = transform.rotated(axis, vel * delta)
    PRISMATIC:
      transform = transform.translated(axis * vel * delta)
  

func toString():
  return "type: %s axis: %s origin: %s" % [type, axis, origin]
