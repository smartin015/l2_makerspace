extends Spatial

var workspace = gamestate.DEFAULT_WORKSPACE
onready var origin = $OQ_ARVROrigin
onready var head = $OQ_ARVROrigin/OQ_ARVRCamera
onready var left = $OQ_ARVROrigin/OQ_LeftController
onready var right = $OQ_ARVROrigin/OQ_RightController
onready var lgrab = $OQ_ARVROrigin/OQ_LeftController/Feature_StaticGrab
onready var rgrab = $OQ_ARVROrigin/OQ_RightController/Feature_StaticGrab

# ===================== Networked Multiplayer ==========================
var last_head = Transform()
var last_left = Transform()
var last_right = Transform()

func _multiplayerReady():
  last_head = head.transform
  last_left = left.transform
  last_right = right.transform
  
func _multiplayerProcess(delta):
  if gamestate.is_initialized && head != null && left != null && right != null:
    
    if vr.ovrHandTracking:
      var lorient
      var rorient
      var conf
      conf = vr.ovrHandTracking.get_hand_pose(right.controller_id, lorient);
      if (conf <= 0.0):
        lorient = null
      conf = vr.ovrHandTracking.get_hand_pose(left.controller_id, rorient);
      if (conf <= 0.0):
        rorient = null
      rpc_unreliable("puppet_hands", lorient, rorient)
    
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

remote func set_origin(origin: Vector3):
  transform.origin = origin
  rset("puppet_transform", global_transform)

remote func set_workspace(ws):
  print("Workspace now %s" % ws)
  workspace = ws
  # Update puppet visibility after the new workspace
  # is set.
  for p in gamestate.players.get_children():
    if p == self:
      continue
    p.update_visibility()

# ==================== Control Zone ===================================

var control_zone
const CONTROL_ZONE_NAME = "control_zone"
const ARM_LEN = 0.60 # a guess at the average
const TABLE_HEIGHT = 0.8

export(vr.BUTTON) var control_zone_button = vr.BUTTON.A;

func _set_control_zone(zone):
  if zone == control_zone:
    zone = null
    return
  
  if zone == null and control_zone != null:
    control_zone.get_parent().exit_zone(gamestate.player.name)
  
  control_zone = zone
  if control_zone == null:
    scale = Vector3.ONE
    transform.origin = Vector3.ZERO
  else:
    control_zone.get_parent().enter_zone(gamestate.player.name)

    # Get bounding box for zone - scale so the size of the box is an arm's length
    # and put 
    var zone_size = control_zone.get_children()[0].shape.get_extents()
    scale = Vector3.ONE * (max(zone_size.x, zone_size.z) / ARM_LEN)
    transform.origin = control_zone.global_transform.origin + (Vector3.DOWN * (zone_size.y / 2 + TABLE_HEIGHT * scale.y))
  rset("puppet_transform", global_transform)

func _controllerProcess(_delta):
  if vr.button_just_pressed(control_zone_button) || vr.button_just_pressed(vr.BUTTON.X) || Input.is_action_just_pressed("toggle_control_zone"):
    var overlapping_bodies = left.find_node("GrabArea", true, false).get_overlapping_bodies();
    for b in overlapping_bodies:
      if b.get_parent().has_method('enter_zone'):
        _set_control_zone(b)
        return
    _set_control_zone(null)
  if Input.is_action_just_pressed("toggle_workspace"):
    var ws = int(workspace)
    gamestate.set_workspace(str((ws+1) % 9))

# ==================== Grab logic =====================================
var grabStart

func _grabProcess(_delta):
  if lgrab.is_just_grabbing:
    grabStart = lgrab.grabbed_object.get_parent().translation
  elif lgrab.is_grabbing:
    lgrab.grabbed_object.get_parent().translation = lgrab.delta_position + grabStart

# ===================== Main functions ================================

func _ready():
  _multiplayerReady()

func _process(delta):
  _multiplayerProcess(delta)
  _controllerProcess(delta)
  _grabProcess(delta)
