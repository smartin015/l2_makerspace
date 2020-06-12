extends Spatial

export(vr.BUTTON) var toggle_button = vr.BUTTON.Y;

var idx = 0
var buttons = []

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
    queue_free()

func _update_buttons(i):
  idx = (i + 9) % 9
  for i in range(3):
    buttons[i].text = str(idx+i)

func _ready():
  buttons = [
    find_node("Button1", true, false),
    find_node("Button2", true, false),
    find_node("Button3", true, false)
  ]
  for b in buttons:
        b.connect("pressed", self, "_on_Button_pressed",[b])

func _on_Button_pressed(b):
  gamestate.set_workspace(b.text)
