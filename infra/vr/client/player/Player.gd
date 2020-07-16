extends Spatial

const DEFAULT_RAYCAST_LEN = 5.0
var ws = workspace.DEFAULT
var alias setget _noset_alias, _get_alias
onready var origin = $OQ_ARVROrigin
onready var head = $OQ_ARVROrigin/OQ_ARVRCamera
onready var left = $OQ_ARVROrigin/OQ_LeftController
onready var lhand = $OQ_ARVROrigin/OQ_LeftController/Feature_HandModel_Left
onready var rhand = $OQ_ARVROrigin/OQ_RightController/Feature_HandModel_Right
onready var right = $OQ_ARVROrigin/OQ_RightController
onready var lgrab = $OQ_ARVROrigin/OQ_LeftController/Feature_StaticGrab
onready var rgrab = $OQ_ARVROrigin/OQ_RightController/Feature_StaticGrab
onready var redIndicator = $OQ_ARVROrigin/OQ_RightController/RedIndicator

func _noset_alias(_a):
  pass
func _get_alias():
  return gamestate.config["alias"]

# ===================== Networked Multiplayer ==========================
var last_head = Transform()
var last_left = Transform()
var last_right = Transform()
var lorient
var rorient
      
func _multiplayerReady():
  last_head = head.transform
  last_left = left.transform
  last_right = right.transform
  vr.log_info("Multiplayer ready")

  
func _multiplayerProcess(delta):
  # TODO send less data (deltas?)
  if gamestate.is_initialized && head != null && left != null && right != null:
    if vr.ovrHandTracking:
      redIndicator.visible = false
      if rhand.tracking_confidence > 0.0:
        rorient = rhand._vrapi_bone_orientations
      else:
        rorient = null
      if lhand.tracking_confidence > 0.0:
        lorient = lhand._vrapi_bone_orientations
      else:
        lorient = null
      rpc_unreliable("puppet_hands", lorient, rorient)
    else:
      redIndicator.visible = true
      
    
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

remote func set_origin(orig: Vector3):
  transform.origin = orig
  rset("puppet_transform", global_transform)

remote func set_workspace(new_ws):
  print("Workspace now %s" % new_ws)
  self.ws = new_ws
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
  if (vr.button_just_pressed(vr.BUTTON.A) ||
      vr.button_just_pressed(vr.BUTTON.X) ||
      Input.is_action_just_pressed("toggle_control_zone")):
    var overlapping_bodies = left.find_node("GrabArea", true, false).get_overlapping_bodies();
    for b in overlapping_bodies:
      if b.get_parent().has_method('enter_zone'):
        _set_control_zone(b)
        return
    _set_control_zone(null)

# ==================== Grab logic =====================================
var grabStart

func _grabProcess(_delta):
  if lgrab.is_just_grabbing:
    grabStart = lgrab.grabbed_object.get_parent().translation
  elif lgrab.is_grabbing:
    lgrab.grabbed_object.get_parent().translation = lgrab.delta_position + grabStart

# ===================== Controls & Menus ================================

var menuEnabled = true
var lastMenuAction = false
func _menuProcess(_delta):
  var pressed = (vr.button_pressed(vr.BUTTON.Y) || Input.is_action_pressed("ui_cycle_mode"))
  if lastMenuAction:
    if pressed:
      return
    else:
      lastMenuAction = false
  if menuEnabled and not lastMenuAction and pressed:
    var tf = head.global_transform
    tf = tf.translated(Vector3(0, 0, -0.6))
    gamestate.toggle_menu(tf)
    lastMenuAction = true

func set_raycast_len(l: float):
  var uirc = find_node("Feature_UIRayCast", true, false)
  if uirc != null:
    uirc.ui_raycast_length = l
    uirc.ui_mesh_length = l

const HAND_CONTROL_DEBOUNCE = 2000
var lastHandControl = 0
var handControlActive = false
var lgest = null
var rgest = null
onready var raycast = $OQ_ARVROrigin/OQ_RightController/Feature_UIRayCast
func _handControlProcess(_delta):
  # Based on hand gestures in both hands, toggle
  # the more "sensitive" controls so the user
  # can gesticulate wildly without being surprised
  # by menus, movement etc. 
  var active = false
  if vr.ovrHandTracking:
    lgest = lhand.detect_simple_gesture()
    rgest = rhand.detect_simple_gesture()
  
  if (lgest == "V" and rgest == "V") or Input.is_action_pressed("ui_hand_control"):
    active = true
  
  if not active:
    handControlActive = false
    return

  var now = OS.get_system_time_msecs()
  if (not handControlActive and active
      and now > lastHandControl + HAND_CONTROL_DEBOUNCE):
    handControlActive = true
    lastHandControl = now
    print("Hand control active - toggling sensitive controls")
    
    raycast.active = not raycast.active    
    menuEnabled = not menuEnabled

# ===================== Main functions ================================

func _ready():
  _multiplayerReady()

onready var dbg = $debug
func _process(delta):
  _handControlProcess(delta)
  _multiplayerProcess(delta)
  _controllerProcess(delta)
  _grabProcess(delta)
  _menuProcess(delta)
  dbg.set_label_text("L: %s R: %s M: %s" % [str(lgest), str(rgest), menuEnabled])
    
