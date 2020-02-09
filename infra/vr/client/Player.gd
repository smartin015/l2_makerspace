extends Spatial

onready var origin = $OQ_ARVROrigin
onready var head = $OQ_ARVROrigin/OQ_ARVRCamera
onready var left = $OQ_ARVROrigin/OQ_LeftController
onready var right = $OQ_ARVROrigin/OQ_RightController

# ===================== Networked Multiplayer ==========================
var last_head = Transform()
var last_left = Transform()
var last_right = Transform()

func _multiplayerReady():
  last_head = head.global_transform
  last_left = left.global_transform
  last_right = right.global_transform
  rpc_id(1, "remote_log", "player ready")

func _multiplayerProcess(delta):
  if gamestate.is_initialized && head != null && left != null && right != null:
    rset_unreliable("puppet_motion", [
      head.global_transform.origin,
      head.global_transform.basis.get_rotation_quat(),
      (head.global_transform.origin - last_head.origin) / delta,
      left.global_transform.origin,
      left.global_transform.basis.get_rotation_quat(),
      (left.global_transform.origin - last_left.origin) / delta,
      right.global_transform.origin,
      right.global_transform.basis.get_rotation_quat(),
      (right.global_transform.origin - last_right.origin) / delta,
     ])
  last_head = head.global_transform
  last_left = left.global_transform
  last_right = right.global_transform

# ==================== Control Zone ===================================

var control_zone
const CONTROL_ZONE_NAME = "L2ControlZone"
const ARM_LEN = 0.60 # a guess at the average
const TABLE_HEIGHT = 0.8

export(vr.BUTTON) var control_zone_button = vr.BUTTON.A;

func _set_control_zone(zone):
  if zone == control_zone:
    zone = null
  print("Setting control zone to ", zone)
  # TODO de-select control zone
  
  control_zone = zone
  if control_zone == null:
    scale = Vector3.ONE
    transform.origin = Vector3.ZERO
  else:
    # Get bounding box for zone - scale so the size of the box is an arm's length
    # and put 
    var zone_size = control_zone.get_children()[0].shape.get_extents()
    scale = Vector3.ONE * (max(zone_size.x, zone_size.z) / ARM_LEN)
    transform.origin = control_zone.global_transform.origin + (Vector3.DOWN * (zone_size.y / 2 + TABLE_HEIGHT * scale.y))
  rset("puppet_transform", global_transform)

func _controllerProcess(_delta):
  if vr.button_just_pressed(control_zone_button) || Input.is_action_just_pressed("toggle_control_zone"):
    var overlapping_bodies = left.find_node("GrabArea", true, false).get_overlapping_bodies();
    print("Found ", overlapping_bodies.size(), " overlaps")
    for b in overlapping_bodies:
      print(b.get_parent().name)
      if b.get_parent().name == CONTROL_ZONE_NAME:
        _set_control_zone(b)
        return
    _set_control_zone(null)
    

# ===================== Main functions ================================

func _ready():
  _multiplayerReady()

func _process(delta):
  _multiplayerProcess(delta)
  _controllerProcess(delta)
