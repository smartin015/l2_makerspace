extends Spatial

var control_zone # String passed in to find `zone`
var pos_topic

func oq_can_static_grab(_b, _grab_area, _controller, _overlapping_bodies):
  if control_zone != null:
    return (str(control_zone.get_writer()) == gamestate.player.name)
  return true
  
const MOVE_THRESHOLD_SQUARED = 0.01*0.01
var debounce = 1.0
var lastpos = Vector3.ZERO

func _ready():
  ROSBridge.advertise(pos_topic, "geometry_msgs/msg/Vector3", "pos_adv")

func _process(delta):
  debounce += delta
  if debounce < 0:
    return
  if (transform.origin - lastpos).length_squared() < MOVE_THRESHOLD_SQUARED:
    return
  # ROS is Z-up
  var pos = transform.origin
  ROSBridge.publish(pos_topic, "geometry_msgs/msg/Vector3", {
    "x": pos[0],
    "y": pos[2],
    "z": pos[1] # ROS is Z-up
  }, "pt")
  
  lastpos = pos
  debounce -= 0.5
