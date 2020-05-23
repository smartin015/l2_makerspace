extends Spatial

export(vr.BUTTON) var toggle_button = vr.BUTTON.Y;

# we have this separate to see it in the editor but
# have it hidden on actual start
export var invisible_on_start = true;

var _uis = [null, "BasicUI"]
var _i = 0
var last_press = 0

func _process(_dt):
  if vr.button_just_pressed(toggle_button) || Input.is_action_just_pressed("ui_cycle_mode"):
    # Debounce, because button pressing is buggy
    var now_msec = OS.get_system_time_msecs()
    print(now_msec)
    if last_press > now_msec - 300:
      return
    last_press = now_msec
    
    _i = (_i + 1) % _uis.size()
    
    var prev = _i
    if _uis[_i] == null:
      visible = false
    else:
      visible = true
      if _uis[prev] != null:
        self.find_node(_uis[prev], true, false).visible = false
      self.find_node(_uis[_i], true, false).visible = true

func _ready():
  if (invisible_on_start): 
    visible = false;
