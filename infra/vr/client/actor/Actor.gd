extends Spatial

const PENDANT_NAME = "pendant"
const HINGE_JOINT_NAME = "HingeJoint"
const DEPTH_RENDER_NAME = "RangeFinder"
const CONTROL_ZONE_NAME = "control_zone"

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

func _setupControlZone(zone):
  if not (zone is MeshInstance and zone.mesh is CubeMesh):
    print("ERROR: zone %s has no CubeMesh" % zone.name)
    return
  zone.set_script(ControlZone)
  var cs = CollisionShape.new()
  cs.shape = BoxShape.new()
  cs.shape.extents = zone.mesh.size / 2
  var sb = StaticBody.new()
  sb.add_child(cs)
  zone.add_child(sb)
  print("Control zone created")
  return

func _setupPendant(n, custom):
  # Pendants in SDF are "empty links". We fill them
  # with an implementation-specific control option.
  var inst = Pendant.instance()
  var zone = find_node(custom["control_zone"], true, false)
  if zone == null:
    print("WARNING: no such control zone %s" % custom["control_zone"])
    return null
  
  # Ensure control zone is scripted
  if !zone.has_method('enter_zone'):
    _setupControlZone(zone)
    
  inst.control_zone = zone
  inst.pos_topic = custom["pos_topic"]
  inst.transform = n.transform
  
  _replace(n, inst)
  return inst

func _setupDepthRender(n):
  # TODO: set the stream ID for this depthrender so it streams
  # from the server
  var inst = DepthRender.instance()
  inst.setup("rvl")
  
  # TODO do this in simulation
  inst.translate(Vector3(0, 1, 0))
  inst.rotate(Vector3(1,0,0), -PI/2)
  _replace(n, inst)

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
    # Can identify objects to replace either by their type
    # or by customData if they're a Robot.
    var nodeType = n.data.get("_type")
    var custom = n.data.get("customData")
    if custom != null:
      var s1 = custom.split(",")
      custom = {}
      for kv in s1:
        var s2 = kv.split(":")
        custom[s2[0]] = s2[1]
      nodeType = custom.get("l2")
    match nodeType:
      PENDANT_NAME:
        n = _setupPendant(n, custom)
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
  
