extends Spatial

# Node references 

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

var dir = Vector3()
const MAX_SPEED = 5
var MOUSE_SENSITIVITY = 0.005
onready var camera = $ARVROrigin/ARVRCamera
var controlled_node
var pitch = 0.0
var yaw = 0.0

func keyboardReady():
  Input.set_mouse_mode(Input.MOUSE_MODE_CAPTURED)
  controlled_node = camera

func keyboardProcess(_delta):	
  dir = Vector3()
  var move = Vector3()
  if Input.is_action_just_pressed("switch_control"):
    if controlled_node == camera:
      controlled_node = left
    elif controlled_node == left:
      controlled_node = right
    elif controlled_node == right:
      controlled_node = camera
    else:
      controlled_node = camera
    print("Switched control to %s" % controlled_node.get_name())
  if Input.is_action_pressed("movement_forward"):
    move.z -= 1
  if Input.is_action_pressed("movement_backward"):
    move.z += 1
  if Input.is_action_pressed("movement_left"):
    move.x -= 1
  if Input.is_action_pressed("movement_right"):
    move.x += 1
  if Input.is_action_pressed("movement_up"):
    move.y += 1
  if Input.is_action_pressed("movement_down"):
    move.y -= 1
  move = move.normalized() * MAX_SPEED * _delta
  controlled_node.transform.origin += move.rotated(Vector3(0,1,0), yaw)
  
func _input(event):
  if !interface and event is InputEventMouseMotion and Input.get_mouse_mode() == Input.MOUSE_MODE_CAPTURED: 
    pitch = clamp(pitch - (event.relative.y * MOUSE_SENSITIVITY), -PI, PI)
    yaw = yaw - (event.relative.x * MOUSE_SENSITIVITY)
    camera.set_rotation(Vector3(pitch, 0, 0))
    camera.global_rotate(Vector3(0, 1, 0), yaw)

# ===================== Networked Multiplayer ==========================

var last_head = Transform()
var last_left = Transform()
var last_right = Transform()

signal playerprocess()

# Called when the node enters the scene tree for the first time.
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
    
