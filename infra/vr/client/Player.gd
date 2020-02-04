extends Spatial

onready var origin = $OQ_ARVROrigin
onready var head = $OQ_ARVROrigin/OQ_ARVRCamera
onready var left = $OQ_ARVROrigin/OQ_LeftController
onready var right = $OQ_ARVROrigin/OQ_RightController

# ===================== Networked Multiplayer ==========================
var last_head = Transform()
var last_left = Transform()
var last_right = Transform()

func multiplayerReady():
  last_head = head.transform
  last_left = left.transform
  last_right = right.transform
  rpc_id(1, "remote_log", "player ready")

func multiplayerProcess(delta):
  if gamestate.is_initialized && head != null && left != null && right != null:
    rset_unreliable("puppet_motion", [
      head.transform.origin,
      head.transform.basis.get_rotation_quat(),
      (head.transform.origin - last_head.origin) / delta,
      left.transform.origin,
      left.transform.basis.get_rotation_quat(),
      (left.transform.origin - last_left.origin) / delta,
      right.transform.origin,
      right.transform.basis.get_rotation_quat(),
      (right.transform.origin - last_right.origin) / delta,
     ])
  last_head = head.transform
  last_left = left.transform
  last_right = right.transform

# ===================== Main functions ================================

func _ready():
  multiplayerReady()

func _process(_delta):
  multiplayerProcess(_delta)
    
