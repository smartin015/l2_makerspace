extends "res://addons/gut/test.gd"

var sdf = load("res://addons/sdf/main.gd").new()

const ASSERT_PROPERTIES = [
  "scale",
  "transform",
 ]

func assert_node_deep_eq(got, want):
  assert_eq(typeof(got), typeof(want), "Matching node types")
  if !want.name.begins_with("@"):
    assert_eq(got.name, want.name, "Matching node names")
    
  for ap in ASSERT_PROPERTIES:
    if want.get(ap):
      assert_eq(got.get(ap), want.get(ap), "Matching property %s on %s" % [ap, want.name])

  var wc = want.get_children()
  var gc = got.get_children()
  assert_eq(len(gc), len(wc), "Children of node with name %s length" % want.name)
  
  if len(wc) != len(gc):
    return
    
  for i in range(len(wc)):
    assert_node_deep_eq(gc[i], wc[i])
  

func test_basic():
  var f = File.new()
  f.open("res://test/unit/basic.sdf", File.READ)
  var t = f.get_as_text()
  var models = sdf.ParseAttrs(t)
  
  var want = Spatial.new()
  want.name = "test_model"
  var wl = Spatial.new()
  wl.name = "test_link"
  wl.transform.origin = Vector3(0.1, 0, 0)
  wl.rotate_x(0.1)
  want.add_child(wl)
  var mi = MeshInstance.new()
  mi.mesh = CubeMesh.new()
  mi.scale = Vector3(0.1, 0.1, 0.1)
  wl.add_child(mi)
  
  assert_eq(len(models.keys()), 1, "exactly one model returned")
  assert_node_deep_eq(models["test_model"], want)

func notest_shapes():
  var f = File.new()
  f.open("res://test/unit/shapes.sdf", File.READ)
  var t = f.get_as_text()
  var models = sdf.ParseAttrs(t)
  
  var want = Spatial.new()
  want.name = "test_model"
  var meshes = {
    "cube": CubeMesh.new(),
    "sphere": SphereMesh.new(),
    "plane": PlaneMesh.new(),
    "cylinder": CylinderMesh.new(),
  }
  for k in meshes.keys():
    var wl = Spatial.new()
    wl.name = k
    want.add_child(wl)
    
    var mi = MeshInstance.new()
    mi.mesh = meshes[k]
    mi.name = k+"_instance"
    match k:
      "cube":
        mi.scale = Vector3(0.1,0.1,0.1)
      "sphere":
        mi.scale = Vector3(0.2,0.2,0.2)
      "plane":
        mi.scale = Vector3(0.1,1.0,0.1)
      "cylinder":
        mi.scale = Vector3(0.1,0.1,0.1)
    wl.add_child(mi)

  assert_eq(len(models.keys()), 1, "exactly one model returned")
  assert_node_deep_eq(models["test_model"], want)

func test_dolly():
  var f = File.new()
  f.open("res://test/unit/dolly.sdf", File.READ)
  var t = f.get_as_text()
  var models = sdf.ParseAttrs(t)
