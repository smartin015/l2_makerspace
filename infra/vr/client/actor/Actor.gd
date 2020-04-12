extends Spatial

const PENDANT_NAME = "pendant"
const DEPTH_RENDER_NAME = "depth_render"
const CONTROL_ZONE_NAME = "control_zone"
const TYPE_ATTR = "l2type"
const CONTROL_ATTR = "l2control"

var Pendant = load("res://Pendant.tscn")
var DepthRender = load("res://DepthRender.tscn")
var ControlZone = load("res://ControlZone.gd")
var ExampleControlPreset = load("res://ExampleControlPreset.tscn")

enum JointType {
  INVALID,
  REVOLUTE,
  PRISMATIC,  
}

class PuppetJoint:
  # This is different than the typical godot Joint classes, which
  # actually attempt to solve fr joint positions. Here we're simply
  # driving the joint values from elsewhere, dictated by ROS (e.g. gazebo)
  var type: int
  var limits: Vector2
  var axis: Vector3
  var child: Spatial
  var origin: Transform
  func apply(val: int):
    var mapped = ((limits[1] - limits[0]) * val + limits[0])
    match type:
      JointType.REVOLUTE:
        child.transform = origin.rotated(axis, mapped)
      JointType.PRISMATIC:
        child.transform.origin = origin + mapped * axis
      _:
        print("Unknown joint type ", type)

  func toString():
    return "type: %s lim: %s axis: %s origin: %s child: %s" % [type, limits, axis, origin, child.name]

var root = null
var joints = {}

func _postprocess(m: Node):
  for c in m.get_children():
    if c.get_class() != "SDFNode":
      continue
    
    if c.attrs.get(CONTROL_ATTR) != null:
      match c.attrs[CONTROL_ATTR]:
        'test_control_preset':
          var n = ExampleControlPreset.instance()
          c.add_child(n)
    
    if c.attrs.get(TYPE_ATTR) == PENDANT_NAME:
      # Pendants in SDF are "empty links". We fill them
      # with an implementation-specific control option.
      var inst = Pendant.instance()
      c.add_child(inst)
      print("Added pendant")
    elif c.type == "l2_ui":
      print("Found l2_ui")
    elif c.type == "joint":
      var j = PuppetJoint.new()
      match c.attrs["type"]:
        "revolute":
          j.type = JointType.REVOLUTE
        "prismatic":
          j.type = JointType.PRISMATIC
        "fixed":
          pass # Nothing to do
        _:
          print("Unimplemented joint type ", c.attrs["type"])
      var parent = root.find_node(c.get_node("parent").data, true, false)
      var pose = c.get_node("pose").data.split(" ")
      j.origin = parent.transform.translated(
        Vector3(float(pose[0]), float(pose[2]), float(pose[1])))
      j.origin = j.origin.rotated(Vector3.FORWARD, float(pose[3]))
      j.origin = j.origin.rotated(Vector3.UP, float(pose[5]))
      j.origin = j.origin.rotated(Vector3.LEFT, float(pose[4]))
      j.child = root.find_node(c.get_node("child").data, true, false)
      var axis = c.get_node("axis/xyz").data.split(" ")
      j.axis = Vector3(axis[0], axis[2], axis[1])
      j.limits = Vector2(
        float(c.get_node("axis/limit/lower").data), 
        float(c.get_node("axis/limit/upper").data))
      joints[c.attrs["name"]] = j
    elif c.attrs.get(TYPE_ATTR) == DEPTH_RENDER_NAME:
      # TODO: set the stream ID for this depthrender so it streams
      # from the server
      var inst = DepthRender.instance()
      c.add_child(inst)
    elif c.attrs.get(TYPE_ATTR) == CONTROL_ZONE_NAME:
      c.set_script(ControlZone)
    else:
      _postprocess(c)
  return m

func setup_controls():
  root = get_child(0)
  _postprocess(root)
  ROSBridge.ros_connect("/joint_states", 
    "sensor_msgs/JointState", 
    self, "_handle_joint_states", 
    "joint_state_sub")
    
func _update_joints(js):
  for j in js.keys():
    joints[j].apply(js[j])
    print("Set joint ", j, " to ", js[j])
    
func _handle_joint_states(msg, id):
  print(msg)
  if joints.get(msg.get('name')):
    # TODO also use msg.message.velocity
    _update_joints({msg.name: msg.position})
  
