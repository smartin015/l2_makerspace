extends Spatial

onready var rect = $OQ_UI2DCanvas/ReferenceRect
export(vr.BUTTON) var toggle_button = vr.BUTTON.Y;

var idx = 0
var buttons = []

# we have this separate to see it in the editor but
# have it hidden on actual start
export var invisible_on_start = true;

var last_action = 0
func _process(_dt):
  var now_msec = OS.get_system_time_msecs()
  # Debounce inputs
  if last_action > now_msec - 300:
    return
      
  if vr.get_controller_axis(vr.AXIS.LEFT_JOYSTICK_X) > 0.9:
    last_action = now_msec
    _update_buttons(idx + 3)
  elif vr.get_controller_axis(vr.AXIS.LEFT_JOYSTICK_X) < -0.9:
    last_action = now_msec
    _update_buttons(idx - 3)
  if vr.button_just_pressed(toggle_button) || Input.is_action_just_pressed("ui_cycle_mode"):
    last_action = now_msec
    visible = !visible

func _update_buttons(i):
  idx = (i + 9) % 9
  for i in range(3):
    buttons[i].text = str(idx+i)

func _ready():
  if (invisible_on_start): 
    visible = false;
  buttons = [
    find_node("Button1", true, false),
    find_node("Button2", true, false),
    find_node("Button3", true, false)
  ]
  for b in buttons:
        b.connect("pressed", self, "_on_Button_pressed",[b])

func _on_Button_pressed(b):
  gamestate.set_workspace(b.text)
