extends "res://addons/gut/test.gd"

var proto = load("res://addons/parse/proto.gd").new()

func _path_to(node):
  var result = PoolStringArray()
  var p = node
  while p != null:
    if typeof(p) == TYPE_OBJECT and p.get_class() == "L2Node":
      result.insert(0, p.data.get("_type"))
    else:
      var v = p.get("name")
      if v == null:
        v = ""
      result.insert(0, p.get_class() + "(" + v + ")")
    if p.get("get_parent"):
      p = p.get_parent()
    else:
      p = null
  return result.join("/")

const ASSERT_PROPERTIES = [
  "scale",
  "transform",
  "size",
  "radius",
  "height",
  "albedo_color",
  "roughness",
  "metallic",
 ]
const EDGE_PROPERTIES = [
  "mesh",
  "material_override",
 ]
func assert_node_deep_eq(got, want):
  assert_eq(typeof(got), typeof(want), "Matching node types")
  if want.get(name) and !want.name.begins_with("@"):
    assert_eq(got.name, want.name, "Matching node names")
    
  for ap in ASSERT_PROPERTIES:
    if want.get(ap):
      assert_eq(got.get(ap), want.get(ap), "Matching property %s on %s" % [ap, _path_to(want)])

  var wc = []
  var gc = []
  if want.has_method("get_children"):
    wc = want.get_children()
    gc = got.get_children()
  for ep in EDGE_PROPERTIES:
    if want.get(ep):
      print("adding edge prop " + ep)
      wc.append(want.get(ep))
      gc.append(got.get(ep))
  assert_eq(len(gc), len(wc), "Children of node %s length" % _path_to(want))
  
  if len(wc) != len(gc):
    return
    
  for i in range(len(wc)):
    assert_node_deep_eq(gc[i], wc[i])

func test_basic_world():
  var got = proto.ParseAttrs("""
  #VRML_SIM V8.5 utf8
  WorldInfo {
  }
  Viewpoint {
    orientation 1 0 0 -0.8
    position 0.25 0.708035 0.894691
  }
  Background {
    skyColor [0.2 0.2 0.2]
  }
  Solid {
    translation 0 1 0
    children [
      Shape {
        appearance PBRAppearance {
          baseColor 0 1 0
          roughness 0.2
          metalness 0
        }
        geometry Box {
          size 0.23 0.1 0.1
        }
      }
    ]
  }""")
    
  var want = Spatial.new()
  var wb = Spatial.new()
  wb.transform.origin = Vector3(0, 1, 0)
  want.add_child(wb)
  var mi = MeshInstance.new()
  mi.mesh = CubeMesh.new()
  mi.mesh.size = Vector3(0.23, 0.1, 0.1)
  mi.material_override = SpatialMaterial.new()
  mi.material_override.roughness = 0.2
  mi.material_override.metallic = 0
  mi.material_override.albedo_color = Color(0, 1, 0)
  wb.add_child(mi)
  assert_node_deep_eq(got, want)

func test_basic_robot():
  var got = proto.ParseAttrs("""
  Robot {
    children [
      HingeJoint {
        jointParameters HingeJointParameters {
          anchor 0 1 0
        }
        device [
          RotationalMotor {
            name "wheel1"
          }
        ]
        endPoint Solid {
          translation 0 1 0
          rotation 0 1 0 0
          children [
            Shape {
              appearance PBRAppearance {
                baseColor 1 1 1
                roughness 1
                metalness 0
              }
              geometry Box {size 0.1 0.1 0.1}
            }
          ]
        }
      }
    ]
  }""")
  var dbg = JSON.print(got.debug())
  # assert(dbg == "")
  var want = Spatial.new()
  var jb = Spatial.new()
  jb.transform.origin = Vector3(0, 1, 0)
  want.add_child(jb)
  var mi = MeshInstance.new()
  mi.mesh = CubeMesh.new()
  mi.mesh.size = Vector3(0.23, 0.1, 0.1)
  mi.material_override = SpatialMaterial.new()
  mi.material_override.roughness = 0.2
  mi.material_override.metallic = 0
  mi.material_override.albedo_color = Color(0, 1, 0)
  jb.add_child(mi)
  
  var root = Spatial.new()
  root.add_child(want)
  # assert_node_deep_eq(got, want)
  
func test_proto_def_and_load():
  pass
