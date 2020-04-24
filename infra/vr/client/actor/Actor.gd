extends Spatial

const PENDANT_NAME = "pendant"
const HINGE_JOINT_NAME = "HingeJoint"
const DEPTH_RENDER_NAME = "RangeFinder"
const CONTROL_ZONE_NAME = "control_zone"
const TYPE_ATTR = "_type"

var Pendant = load("res://actor/Pendant.tscn")
var DepthRender = load("res://actor/DepthRender.tscn")
var ControlZone = load("res://actor/ControlZone.gd")
var PuppetJoint = load("res://actor/PuppetJoint.gd")
var ExampleControlPreset = load("res://ExampleControlPreset.tscn")

func _replace(old, new):
  old.get_parent().add_child(new)
  for c in old.get_children():
    old.remove_child(c)
    new.add_child(c)
  old.free()

func _setupPendant(n):
  # Pendants in SDF are "empty links". We fill them
  # with an implementation-specific control option.
  var inst = Pendant.instance()
  n.add_child(inst)
  print("Added pendant")

func _setupDepthRender(n):
  # TODO: set the stream ID for this depthrender so it streams
  # from the server
  var inst = DepthRender.instance()
  inst.setup(16, 16, "rvl")
  
  # TODO remove
  inst.translate(Vector3(0, 1, 0))
  
  _replace(n, inst)

func _setupControlZone(n):
  n.set_script(ControlZone)

func _setupHingeJoint(n):
  var j = PuppetJoint.new()
  if n.data.get("name") == null:
    print("ERROR joint has no name! data " + str(n.data))
    return
  j.type = PuppetJoint.REVOLUTE
  var axis = n.data.get("axis", [0,1,0])
  j.axis = Vector3(axis[0], axis[2], axis[1]).normalized()
  joints[n.data.get("name")] = j
  _replace(n, j)
  return j

var root = null
var joints = {}
func _postprocess(n: Node):
  if n.get_class() == "L2Node":
    match n.data.get(TYPE_ATTR):
      PENDANT_NAME:
        _setupPendant(n)
      HINGE_JOINT_NAME:
        n = _setupHingeJoint(n)
      DEPTH_RENDER_NAME:
        n = _setupDepthRender(n)
      CONTROL_ZONE_NAME:
        _setupControlZone(n)
  if n == null:
    return
  for c in n.get_children():
    _postprocess(c)
  return n

func setup_controls():
  root = get_child(0)
  _postprocess(root)
  ROSBridge.ros_connect("joint_states", 
    "sensor_msgs/JointState", 
    self, "_handle_joint_states", 
    "joint_state_sub")
    
func _handle_joint_states(msg, id):
  for i in len(msg['name']):
    var j = joints.get(msg['name'][i])
    if j != null:
      j.apply(msg["position"][i], msg["velocity"][i])
  
