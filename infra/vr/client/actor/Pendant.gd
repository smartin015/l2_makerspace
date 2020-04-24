extends Spatial

func oq_can_static_grab(_b, _grab_area, _controller, _overlapping_bodies):
  var zone = find_parent("L2ControlZone")
  if zone != null:
    return (zone.writer == gamestate.my_name)
  return true
  
const MOVE_THRESHOLD_SQUARED = 0.01*0.01
var debounce = 1.0
var lastpos = Vector3.ZERO

func _process(delta):
  debounce += delta
  if debounce < 0:
    return
  if (transform.origin - lastpos).length_squared() < MOVE_THRESHOLD_SQUARED:
    return
  # ROS is Z-up
  var pos = transform.origin
  ROSBridge.publish(get_path().get_concatenated_subnames() + ":pos", "std_msgs/Vector3", {
    "x": pos[0],
    "y": pos[2],
    "z": pos[1]
   }, "pt")
  
  lastpos = pos
  debounce -= 0.5
