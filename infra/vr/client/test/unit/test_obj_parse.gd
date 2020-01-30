extends "res://addons/gut/test.gd"

var objparse = load("res://addons/mesh/ObjParse.gd").new()

func _triangle_area(vs):
  # Heron's formula
  var a = (vs[0] - vs[1]).length()
  var b = (vs[0] - vs[2]).length()
  var c = (vs[1] - vs[2]).length()
  var s = 0.5 * (a + b + c)
  return sqrt(s * (s - a) * (s - b) * (s - c))

func assert_mesh_eq(got: Mesh, want: Mesh, want_surface_area):
  var gf = got.get_faces()
  var wf = want.get_faces()
  # Because the ordering of tris varies between meshes, we simply check
  # that the number of face vertices and the total surface area matches
  if gf.size() != wf.size():
    assert_eq(gf.size(), wf.size(), "face vertex count matches")
    return
  var gArea = 0.0
  for i in range(len(gf)/3):
    gArea += _triangle_area([gf[3*i], gf[3*i+1], gf[3*i+2]])
  assert_eq(round(gArea), round(want_surface_area), "face surface area")
  if got.get_surface_count() != want.get_surface_count():
    assert_eq(got.get_surface_count(), want.get_surface_count(), "surface count matches")
    return
  for s in range(got.get_surface_count()):
    var gs = got.surface_get_arrays(s)
    var ws = got.surface_get_arrays(s)
    if gs.size() != ws.size():
      assert_eq(gs.size(), ws.size(), "surface arrays count matches, surface %d" % s)
      return
    for i in range(gs.size()):
      if gs[i] == null and ws[i] == null:
        continue 
      var g = Array(gs[i])
      var w = Array(ws[i])
      g.sort()
      w.sort()
      if g.size() != w.size():
        assert_eq(g.size(), w.sie(), "surface array %d len matches, surface %d" % [i, s])
        return
      for j in range(len(gs[i])):
        assert_eq(gs[i][j], ws[i][j], "surface %d array %d index %d" % [s, i, j])

func do_test(objpath, mesh, surface_area):
  var f = File.new()
  var err = f.open(objpath, File.READ)
  assert_eq(err, OK, "open file")
  var got = objparse.parse(f.get_as_text())
  assert_mesh_eq(got, mesh, surface_area)

func test_plane():
  do_test("res://test/unit/plane.obj", PlaneMesh.new(), 4)
  
func test_cube():
  do_test("res://test/unit/cube.obj", CubeMesh.new(), 24)
