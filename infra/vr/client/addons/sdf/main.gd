
var xml

const NODE_MODEL = "model"
const NODE_LINK = "link"
const MESH_CUBE = "cube"
const MESH_CYLINDER = "cylinder"
const MESH_PLANE = "plane"
const MESH_SPHERE = "sphere"

func upsertMesh(link: Node, type: String) -> MeshInstance:
  var n = link.get_node_or_null("meshinstance")
  if n != null:
    return n
  n = MeshInstance.new()
  n.name = "meshinstance"
  n.material_override = SpatialMaterial.new()
  match type:
    MESH_CUBE:
      n.mesh = CubeMesh.new()
    MESH_CYLINDER:
      n.mesh = CylinderMesh.new()
    MESH_PLANE:
      n.mesh = PlaneMesh.new()
    MESH_SPHERE:
      n.mesh = SphereMesh.new()
    _:
      print("Error: Unknown type ", type)
  link.add_child(n)
  return n

func addCollisionShape(link: Node, extents: Vector3):
  var sb = StaticBody.new()
  var n = CollisionShape.new()
  n.name = link.name 
  n.shape = BoxShape.new()
  n.shape.extents = extents
  sb.add_child(n)
  link.add_child(sb)

func handleAttribute(model: Spatial, link: Spatial, p, data):
  match p:
    "/pose":
      var v = data.split(" ", false)
      link.transform.origin = Vector3(float(v[0]), float(v[2]), float(v[1]))
      link.rotate_x(float(v[3]))
      link.rotate_y(float(v[4]))
      link.rotate_z(float(v[5]))
    "/visual/transparency":
      var mi = link.get_node("meshinstance")
      mi.material_override.flags_transparent = true
      mi.material_override.albedo_color.a = float(data)
    "/visual/material/script":
      var mi = link.get_node("meshinstance")
      match data:
        "Gazebo/Purple":
          mi.material_override.albedo_color = Color(0.5, 0, 0.5)
        _:
          print("Unknown color ", data)
    "/visual/geometry/box/size":
      var mi = upsertMesh(link, MESH_CUBE)
      var v = data.split(" ", false)
      mi.scale = Vector3(float(v[0]), float(v[2]), float(v[1]))
    "/collision/geometry/box/size":
      var v = data.split(" ", false)
      addCollisionShape(link, Vector3(float(v[0]), float(v[2]), float(v[1])))
      print("Added collision shape")
    "/visual/geometry/cylinder/radius":
      var mi = upsertMesh(link, MESH_CYLINDER)
      var r = float(data)
      mi.scale.x = r
      mi.scale.z = r
    "/visual/geometry/cylinder/length":
      var mi = upsertMesh(link, MESH_CYLINDER)
      var l = float(data)
      mi.scale.y = l
    "/visual/geometry/plane/normal":
      var mi = upsertMesh(link, MESH_PLANE)
      var v = data.split(" ", false)
      var n = Vector3(float(v[0]), float(v[2]), float(v[1]))
      if n.angle_to(Vector3.UP) > 0:
        var axis = n.cross(Vector3(0,1,0)).normalized()
        mi.rotate(axis, n.angle_to(Vector3.UP))
    "/visual/geometry/plane/size":
      var mi = upsertMesh(link, MESH_PLANE)
      var v = data.split(" ", false)
      mi.scale = Vector3(float(v[0]), 1, float(v[1]))
    "/visual/geometry/sphere/radius":
      var mi = upsertMesh(link, MESH_SPHERE)
      var d = 2*float(data)
      mi.scale = Vector3(d, d, d)
    _:
      print(p, " ", data, " (ignored)")

func relPath(path: PoolStringArray) -> String:
  var p = ""
  var i = 0
  var refs = 0
  while i < len(path):
    if refs >= 2:
      p += "/" + path[i]
    if path[i].begins_with(NODE_MODEL) || path[i].begins_with(NODE_LINK):
      refs += 1
    i += 1
  return p

func ParseAttrs(xmlstr: String):
  xml = XMLParser.new()
  var err = xml.open_buffer(xmlstr.to_ascii())
  if err != OK:
    return err
    
  err = xml.read()  
  if xml.get_node_name().begins_with("?xml"):
    err = xml.read()

  var path = PoolStringArray()
  var models = {}
  var model
  var link
  var joints = {}
  # TODO switch to named spatials for all nodes
  while err == OK:
    if xml.get_node_type() == XMLParser.NODE_ELEMENT:
      if xml.has_attribute("name"):
        var name = xml.get_named_attribute_value("name")
        print(name)
        path.push_back(xml.get_node_name() + ":" + name)
        match xml.get_node_name():
          NODE_MODEL:
            if !models.has(name):
              model = Spatial.new()
              model.name = name
              models[name] = model
            else:
              model = models[name]
          NODE_LINK:
            if model.find_node(name) == null:
              link = Spatial.new()
              link.name = name
              model.add_child(link)
            else:
              link = model.find_node(name)
      else:
        path.push_back(xml.get_node_name())
    elif xml.get_node_type() == XMLParser.NODE_ELEMENT_END:
      var rm = path[-1]
      if rm.begins_with(NODE_MODEL):
        model = null
      elif rm.begins_with(NODE_LINK):
        link = null
      path.remove(len(path)-1)
    elif xml.get_node_type() == XMLParser.NODE_TEXT && xml.get_node_data().lstrip(" \n") != "":
      var data = xml.get_node_data()
      handleAttribute(model, link, relPath(path), data)
      
    err = xml.read()
  
  if err == ERR_FILE_EOF:
    return models
  return err
  
