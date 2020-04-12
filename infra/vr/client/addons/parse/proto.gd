extends Node

var tree = load("res://addons/parse/proto_tree.gd").new()

func _set_geometry(link: Spatial, n):
  var mi = MeshInstance.new()
  mi.material_override = SpatialMaterial.new()
  link.add_child(mi)
  match n.type:
    "Box":
      mi.mesh = CubeMesh.new()
      var v = n.get_node("size").data.split(" ", false)
      mi.scale = Vector3(float(v[0]), float(v[2]), float(v[1]))
    "Plane":
      mi.mesh = PlaneMesh.new()
      var v = n.get_node("normal").data.split(" ", false)
      var norm  = Vector3(float(v[0]), float(v[2]), float(v[1]))
      if norm.angle_to(Vector3.UP) > 0:
        var axis = norm.cross(Vector3(0,1,0)).normalized()
        mi.rotate(axis, n.angle_to(Vector3.UP))
      
      v = n.get_node("size").data.split(" ", false)
      mi.scale = Vector3(float(v[0]), 1, float(v[1]))
    "Cylinder":
      mi.mesh = CylinderMesh.new()
      var r = float(n.get_node("radius").data)
      mi.scale.x = r
      mi.scale.z = r
      var l = float(n.get_node("length").data)
      mi.scale.y = l
    "Sphere":
      mi.mesh = SphereMesh.new()
      var d = 2*float(n.get_node("radius").data)
      mi.scale = Vector3(d, d, d)
    _:
      print("Error: Unknown geometry type ", n.type)

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
    print("Albedo " + str(n2.material_override.albedo_color))

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
  parent.add_child(n2)
  n.queue_free()
  

func _fillSolid(parent, n):
  var t = n.data.get("translation")
  # Note: godot and webots are both Y-up
  if t != null:
    n.transform.origin = Vector3(t[0], t[1], t[2])
  t = n.data.get("rotation")
  if t != null:
    n.rotate(Vector3(t[0], t[1], t[2]), t[3])
  for c in n.get_children():
    _fillNode(n, c)

func _fillNode(parent, n):
  match n.data["_type"]:
    "Solid":
      _fillSolid(parent, n)
    "Shape":
      _fillShape(parent, n)
    _:
      print("Ignoring: " + n.data["_type"])

func ParseAttrs(s):
  # TODO handle PROTO loading n'at
  # if s[0] == "#": # We got a world file
  tree.init()
  var root = tree.parse(s)
  for c in root.get_children():
    _fillNode(root, c)
  print(root.debug())
  return root
