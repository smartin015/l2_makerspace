extends Node

var tree = load("res://addons/parse/proto_tree.gd").new()

func _fillShape(parent, n):
  var n2 = MeshInstance.new()
  n2.material_override = SpatialMaterial.new()
  var a = n.data.get("appearance")
  if a != null:
    a = a.data
    var v = a.get("baseColor", [1,1,1])
    var t = a.get("transparency", 0)
    n2.material_override.albedo_color = Color(v[0], v[1], v[2], 1.0-t)
    n2.material_override.roughness = a.get("roughness", 0)
    n2.material_override.metallic = a.get("metalness", 0)

  var g = n.data.get("geometry")
  if g != null:
    var shape
    match g.data["_type"]:
      "Box":
        n2.mesh = CubeMesh.new()
        var v = g.data.get("size", [1,1,1])
        n2.mesh.size = Vector3(v[0], v[1], v[2])
      "Capsule":
        n2.mesh = CapsuleMesh.new()
        n2.mesh.radius = g.data.get("radius", 1)
        n2.mesh.mid_height = g.data.get("height", 1)
      "Cone":
        n2.mesh = CylinderMesh.new()
        n2.mesh.top_radius = 0
        n2.mesh.bottom_radius = g.data.get("radius", 1)
        n2.mesh.height = g.data.get("height", 1)
      "Cylinder":
        n2.mesh = CylinderMesh.new()
        n2.mesh.top_radius = g.data.get("radius", 1)
        n2.mesh.bottom_radius = n2.mesh.top_radius
        n2.mesh.height = g.data.get("height", 1)
      "Plane":
        n2.mesh = PlaneMesh.new()
        var v = g.data.get("size", [1,1])
        n2.mesh.size = Vector2(v[0], v[1])
      "Sphere":
        n2.mesh = SphereMesh.new()
        n2.mesh.radius = g.data.get("radius", 1)
        n2.mesh.height = n2.mesh.radius*2
      _:
        print("WARNING: Unknown shape " + g.data["_type"])
  # Put the shape in place; remove extra bits
  n.get_parent().add_child(n2)
  n.free()
  
func _fillSolid(parent, n):
  var t = n.data.get("translation")
  if t != null:
    n.transform.origin = Vector3(t[0], t[1], t[2])
  t = n.data.get("rotation")
  if t != null:
    n.rotate(Vector3(t[0], t[1], t[2]), t[3])
  for c in n.get_children():
    _fillNode(n, c)

func _fillRobot(parent, n):
  # TODO get robot topics via customData
  var t = n.data.get("translation")
  if t != null:
    n.transform.origin = Vector3(t[0], t[1], t[2])
  t = n.data.get("rotation")
  if t != null:
    n.rotate(Vector3(t[0], t[1], t[2]), t[3])
  for c in n.get_children():
    _fillNode(n, c)

func _fillHingeJoint(parent, n):
  # Clear any child nodes; we don't need them
  #  for c in n.get_children():
  #    c.free()
  var jp = n.data.get("jointParameters")
  var ep = n.data.get("endPoint")
  var dv = n.data.get("device", [])
  n.data = {"_type": "HingeJoint"}
  # https://cyberbotics.com/doc/reference/hingejointparameters
  # TODO also anchor
  if jp != null:
    var v = jp.data.get("axis", [1,0,0])
    n.data["axis"] = Vector3(v[0], v[1], v[2])
  for c in dv:
    # Get motor name
    if c.data["_type"] == "RotationalMotor":
      n.data["name"] = c.data.get("name")
    c.free()
  if ep != null:
    n.add_child(ep)
    _fillNode(n, ep)
  else: 
    print("WARNING no endPoint")
  
func _fillNode(parent, n):
  match n.data["_type"]:
    "Solid":
      _fillSolid(parent, n)
    "Shape":
      _fillShape(parent, n)
    "Robot":
      _fillRobot(parent, n)
    "HingeJoint":
      _fillHingeJoint(parent, n)
    _:
      print("PROTO: ignored " + n.data["_type"])
      n.free()

func ParseAttrs(s):
  # TODO handle PROTO loading n'at
  # if s[0] == "#": # We got a world file
  tree.init()
  var root = tree.parse(s)
  # print(JSON.print(root.debug()))
  for c in root.get_children():
    _fillNode(root, c)
  # print(JSON.print(root.debug()))
  return root
