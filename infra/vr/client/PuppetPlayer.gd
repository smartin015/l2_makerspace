# Credit 
# https://poly.google.com/view/f8iZm3Vvfdm
# for the headset mesh

extends Spatial

puppet var puppet_transform
puppet var puppet_motion

onready var left = $LeftHand
onready var right = $RightHand
onready var head = $Head 

const PM_HEAD = 0
const PM_LEFT = 3
const PM_RIGHT = 6

const PM_ORIGIN = 0
const PM_QUAT = 1
const PM_VEL = 2

func _ready():
  puppet_motion = [
    Vector3.ZERO, Quat.IDENTITY, Vector3.ZERO, # Head
    Vector3.ZERO, Quat.IDENTITY, Vector3.ZERO, # Left
    Vector3.ZERO, Quat.IDENTITY, Vector3.ZERO, # Right
  ]
  puppet_transform = transform

func _process(delta):
  head.transform = Transform(puppet_motion[PM_HEAD+PM_QUAT], puppet_motion[PM_HEAD+PM_ORIGIN])
  head.transform = head.transform.translated(puppet_motion[PM_HEAD+PM_VEL] * delta)
  left.transform = Transform(puppet_motion[PM_LEFT+PM_QUAT], puppet_motion[PM_LEFT+PM_ORIGIN])
  left.transform = left.transform.translated(puppet_motion[PM_LEFT+PM_VEL] * delta)
  right.transform = Transform(puppet_motion[PM_RIGHT+PM_QUAT], puppet_motion[PM_RIGHT+PM_ORIGIN])
  right.transform = right.transform.translated(puppet_motion[PM_RIGHT+PM_VEL] * delta)
  
  # Many frames may pass before the controlling player sends more motion.
  # Updating position minimizes jitter problems
  puppet_motion[PM_HEAD+PM_ORIGIN] = head.transform.origin
  puppet_motion[PM_LEFT+PM_ORIGIN] = left.transform.origin
  puppet_motion[PM_RIGHT+PM_ORIGIN] = right.transform.origin
  if transform != puppet_transform:
    transform = puppet_transform
