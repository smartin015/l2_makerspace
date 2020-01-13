extends Spatial

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
var MOUSE_SENSITIVITY = 0.15
onready var camera = $ARVROrigin/ARVRCamera

func keyboardReady():
	Input.set_mouse_mode(Input.MOUSE_MODE_CAPTURED)

func keyboardProcess(_delta):	
	dir = Vector3()
	var cam_xform = camera.get_global_transform()
	var input_movement_vector = Vector3()
	if Input.is_action_pressed("movement_forward"):
		input_movement_vector.y += 1
	if Input.is_action_pressed("movement_backward"):
		input_movement_vector.y -= 1
	if Input.is_action_pressed("movement_left"):
		input_movement_vector.x -= 1
	if Input.is_action_pressed("movement_right"):
		input_movement_vector.x += 1
	if Input.is_action_pressed("movement_up"):
		input_movement_vector.z += 1
	if Input.is_action_pressed("movement_down"):
		input_movement_vector.z -= 1
	input_movement_vector = input_movement_vector.normalized()
	dir += -cam_xform.basis.z * input_movement_vector.y
	dir += cam_xform.basis.x * input_movement_vector.x
	dir += cam_xform.basis.y * input_movement_vector.z
	transform = transform.translated(dir * MAX_SPEED * _delta)
	
func _input(event):
	if !interface and event is InputEventMouseMotion and Input.get_mouse_mode() == Input.MOUSE_MODE_CAPTURED: 
		self.rotate_x(deg2rad(event.relative.y * MOUSE_SENSITIVITY))
		self.rotate_y(deg2rad(event.relative.x * MOUSE_SENSITIVITY * -1))

# ===================== Networked Multiplayer ==========================

var last_transform = Transform()
var vel = Vector3()

# Called when the node enters the scene tree for the first time.
func multiplayerReady():
	# $NameLabel.text = "You"
	last_transform = transform

func multiplayerProcess(_delta):
	if gamestate.is_initialized && (vel != Vector3.ZERO || last_transform != transform):
		rset_unreliable("puppet_transform", transform)
		vel =  (transform.origin - last_transform.origin)/_delta
		rset_unreliable("puppet_vel", vel)
	last_transform = transform

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
		
