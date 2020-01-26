extends Spatial

onready var origin = $ARVROrigin
onready var head = $ARVROrigin/ARVRCamera
onready var left = $ARVROrigin/LeftHand
onready var right = $ARVROrigin/RightHand

# ===================== VR Control / Initialization ====================

onready var interface = ARVRServer.find_interface("OVRMobile")
var ovr_init_config
var ovr_performance
var perform_runtime_config = false

func ovrReady():
  ovr_init_config = preload("res://addons/godot_ovrmobile/OvrInitConfig.gdns").new()
  ovr_performance = preload("res://addons/godot_ovrmobile/OvrPerformance.gdns").new()
  ovr_init_config.set_render_target_size_multiplier(1)
  if interface.initialize():
    get_viewport().arvr = true

func ovrProcess(_delta):
  if !perform_runtime_config:
    ovr_performance.set_clock_levels(1, 1)
    ovr_performance.set_extra_latency_mode(1)
    perform_runtime_config = true

# ===================== Keyboard/Mouse control =========================
const MAX_SPEED = 5
const MOUSE_SENSITIVITY = 0.005
const ARM_LEN = 1.0
onready var camera = $ARVROrigin/ARVRCamera
var pitch = 0.0
var yaw = 0.0

signal spawn_cube_request()

var NAV_MAP = {
  "movement_forward":  Vector3.FORWARD,
  "movement_backward": Vector3.BACK,
  "movement_left":     Vector3.LEFT,
  "movement_right":    Vector3.RIGHT,
  "movement_up":       Vector3.UP,
  "movement_down":     Vector3.DOWN,
}

func keyboardReady():
  Input.set_mouse_mode(Input.MOUSE_MODE_CAPTURED)

func keyboardProcess(_delta):	
  var move = Vector3()
  for k in NAV_MAP.keys():
    if Input.is_action_pressed(k):
      move += NAV_MAP[k]
  if Input.is_action_just_pressed("spawn_cube"):
    emit_signal("spawn_cube_request")
  
  move = move.normalized() * MAX_SPEED * _delta
  camera.transform.origin += move.rotated(Vector3.UP, yaw)
  left.transform = camera.transform.translated(Vector3.FORWARD * ARM_LEN)
  right.transform = camera.transform
  
func _input(event):
  if !interface and event is InputEventMouseMotion and Input.get_mouse_mode() == Input.MOUSE_MODE_CAPTURED: 
    pitch = clamp(pitch - (event.relative.y * MOUSE_SENSITIVITY), -PI, PI)
    yaw = yaw - (event.relative.x * MOUSE_SENSITIVITY)
    camera.set_rotation(Vector3(pitch, 0, 0))
    camera.global_rotate(Vector3.UP, yaw)

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
  if interface:
    ovrReady()
  else:
    keyboardReady()
  multiplayerReady()

func _process(_delta):
  multiplayerProcess(_delta)
  if interface:
    ovrProcess(_delta)
  else:
    keyboardProcess(_delta)
    
