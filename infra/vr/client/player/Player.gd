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
var lorient
var rorient
      
func _multiplayerReady():
  last_head = head.transform
  last_left = left.transform
  last_right = right.transform
  vr.log_info("Multiplayer ready")
  
# This is a test pose for the left hand used only on desktop so the hand has a proper position
# TODO remove after testing
const test_pose_left_ThumbsUp = [Quat(0, 0, 0, 1), Quat(0, 0, 0, 1), Quat(0.321311, 0.450518, -0.055395, 0.831098),
Quat(0.263483, -0.092072, 0.093766, 0.955671), Quat(-0.082704, -0.076956, -0.083991, 0.990042),
Quat(0.085132, 0.074532, -0.185419, 0.976124), Quat(0.010016, -0.068604, 0.563012, 0.823536),
Quat(-0.019362, 0.016689, 0.8093, 0.586839), Quat(-0.01652, -0.01319, 0.535006, 0.844584),
Quat(-0.072779, -0.078873, 0.665195, 0.738917), Quat(-0.0125, 0.004871, 0.707232, 0.706854),
Quat(-0.092244, 0.02486, 0.57957, 0.809304), Quat(-0.10324, -0.040148, 0.705716, 0.699782),
Quat(-0.041179, 0.022867, 0.741938, 0.668812), Quat(-0.030043, 0.026896, 0.558157, 0.828755),
Quat(-0.207036, -0.140343, 0.018312, 0.968042), Quat(0.054699, -0.041463, 0.706765, 0.704111),
Quat(-0.081241, -0.013242, 0.560496, 0.824056), Quat(0.00276, 0.037404, 0.637818, 0.769273),
]
  
func _multiplayerProcess(delta):
  # TODO send less data (deltas?)
  if gamestate.is_initialized && head != null && left != null && right != null:
    if vr.ovrHandTracking:
      if right.param_model.visible:
        rorient = right._vrapi_bone_orientations
      else:
        rorient = test_pose_left_ThumbsUp
      if left.param_model.visible:
        lorient = left._vrapi_bone_orientations
      else:
        lorient = test_pose_left_ThumbsUp
      gamestate.shout(str(len(lorient)))
      rpc_unreliable("puppet_hands", lorient, rorient)
    else:
      rpc_unreliable("puppet_hands", null, null)
    
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

# ===================== Controls & Menus ================================

func _menuProcess(_delta):
  if vr.button_just_pressed(vr.BUTTON.Y) || Input.is_action_just_pressed("ui_cycle_mode"):
    gamestate.tools.spawn(
      "menu", "MENU", 
      head.transform.translated(Vector3(0, 0, -0.6)),
      true # local
    )

# ===================== Main functions ================================

func _ready():
  _multiplayerReady()

func _process(delta):
  _multiplayerProcess(delta)
  _controllerProcess(delta)
  _grabProcess(delta)
  _menuProcess(delta)
