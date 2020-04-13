const NODE_MODEL = "model"
const NODE_LINK = "link"
const NODE_JOINT = "joint"

# SDFNode is a basic node that contains the XML data for a corresponding SDF element
class SDFNode extends Spatial:
  var type = ""
  var data = ""
  var attrs = {}
  func get_class():
    return "SDFNode"

func ParseAttrs(xmlstr: String):
  var xml = XMLParser.new()
  var err = xml.open_buffer(xmlstr.to_ascii())
  if err != OK:
    return err
    
  err = xml.read()  
  if xml.get_node_name().begins_with("?xml"):
    err = xml.read()

  # Convert to structured Godot spatial node format
  var root = SDFNode.new()
  var cur = root
  while err == OK:
    if xml.get_node_type() == XMLParser.NODE_ELEMENT:
      var n = SDFNode.new()
      n.name = xml.get_node_name()
      n.type = n.name
      if xml.has_attribute("name"):
        n.name = xml.get_named_attribute_value("name")
      for i in range(xml.get_attribute_count()):
        n.attrs[xml.get_attribute_name(i)] = xml.get_attribute_value(i)
      cur.add_child(n)
      cur = n
    elif xml.get_node_type() == XMLParser.NODE_ELEMENT_END:
      cur = cur.get_parent()
    elif xml.get_node_type() == XMLParser.NODE_TEXT && xml.get_node_data().lstrip(" \n") != "":
      cur.data = xml.get_node_data()   
    err = xml.read()
  
  if err != ERR_FILE_EOF:
    return err
  
  return _fill_sdf(root)

func _set_collision(link: Node, n: SDFNode):
  var sb = StaticBody.new()
  var cs = CollisionShape.new()
  n.name = link.name 
  # TODO different shapes
  cs.shape = BoxShape.new()
  var v = n.get_node("size").data.split(" ", false)
  cs.shape.extents = Vector3(float(v[0]), 1, float(v[1]))
  sb.add_child(cs)
  link.add_child(sb)

func _set_geometry(link: Spatial, n: SDFNode):
  var mi = MeshInstance.new()
  mi.material_override = SpatialMaterial.new()
  link.add_child(mi)
  match n.type:
    "box":
      mi.mesh = CubeMesh.new()
      var v = n.get_node("size").data.split(" ", false)
      mi.scale = Vector3(float(v[0]), float(v[2]), float(v[1]))
    "plane":
      mi.mesh = PlaneMesh.new()
      var v = n.get_node("normal").data.split(" ", false)
      var norm  = Vector3(float(v[0]), float(v[2]), float(v[1]))
      if norm.angle_to(Vector3.UP) > 0:
        var axis = norm.cross(Vector3(0,1,0)).normalized()
        mi.rotate(axis, n.angle_to(Vector3.UP))
      
      v = n.get_node("size").data.split(" ", false)
      mi.scale = Vector3(float(v[0]), 1, float(v[1]))
    "cylinder":
      mi.mesh = CylinderMesh.new()
      var r = float(n.get_node("radius").data)
      mi.scale.x = r
      mi.scale.z = r
      var l = float(n.get_node("length").data)
      mi.scale.y = l
    "sphere":
      mi.mesh = SphereMesh.new()
      var d = 2*float(n.get_node("radius").data)
      mi.scale = Vector3(d, d, d)
    _:
      print("Error: Unknown geometry type ", n.type)

# Replace occurrences of particular tags with standard godot primitives
func _fill_sdf(n: SDFNode):
  for c in n.get_children():
        _fill_sdf(c)
        
  match n.type:
    NODE_LINK:
      var pose = n.get_node("pose")
      if pose != null:
        var v = pose.data.split(" ", false)
        n.transform.origin = Vector3(float(v[0]), float(v[2]), float(v[1]))
        n.rotate_x(float(v[3]))
        n.rotate_y(float(v[4]))
        n.rotate_z(float(v[5]))
      var geom = n.get_node("visual/geometry")
      if geom != null:
        _set_geometry(n, geom.get_child(0))
      var coll = n.get_node("collision/geometry")
      if coll != null:
        _set_collision(n, coll.get_child(0))
      # TODO transparency & color

  return n
