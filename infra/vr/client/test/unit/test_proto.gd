extends "res://addons/gut/test.gd"

var proto = load("res://addons/parse/proto.gd").new()

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

func test_basic_world():
  var world = proto.ParseAttrs("""
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
  }
  Solid {
    translation -0.2 1 0
    children [
      Shape {
        appearance PBRAppearance {
          baseColor 1 0 0
          roughness 0.2
          metalness 0.5
        }
        geometry Sphere {
          radius 0.1
        }
      }
    ]
  }
  Solid {
    translation 0.2 1 0
    children [
      Shape {
        appearance PBRAppearance {
          baseColor 0 0 1
          roughness 1.0
          metalness 0.0
        }
        geometry Cylinder {
          radius 0.1
          height 0.1
        }
      }
    ]
  }""")
  
  var obj = JSON.print(world.debug())
  assert(false)
  #  var want = Spatial.new()
  #  want.name = "test_model"
  #  var wl = Spatial.new()
  #  wl.name = "test_link"
  #  wl.transform.origin = Vector3(0.1, 0, 0)
  #  wl.rotate_x(0.1)
  #  want.add_child(wl)
  #  var mi = MeshInstance.new()
  #  mi.mesh = CubeMesh.new()
  #  mi.scale = Vector3(0.1, 0.1, 0.1)
  #  wl.add_child(mi)
    
  #assert_eq(len(models.keys()), 1, "exactly one model returned")
  #assert_node_deep_eq(models["test_model"], want)
  

func test_proto_def_and_load():
  pass
