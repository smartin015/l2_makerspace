# Made with inspiration by Ezcha's Obj parser
# at https://github.com/Ezcha/gd-obj
extends Node

var mat

const VERTEX = "v"
const NORMAL = "vn"
const TEXTURE = "vt"
const FACE = "f"

const LINE_SEP = "\n"
const PART_SEP = " "
const INDEX_SEP = "/"

func _ready():
  mat = SpatialMaterial.new()
  mat.albedo_color = Color(1, 1, 1)
  mat.flags_transparent = false
  mat.depth_enabled = false

func _split_point_idx(part):
  # subtract to get array indexes instead of OBJ (which starts at 1)
  var vi = part.split(INDEX_SEP)
  return [int(vi[0])-1, int(vi[1])-1, int(vi[2])-1]

func _parse_face(parts: PoolStringArray):
  # A "face" line in OBJ is a sequence
  # f v1/vt1/vn1 v2/vt2/vn2 v3/vt3/vn3... v#/vt#/vn#
  # where v, vt, and vn indexes into the set of vertices, textures, and normals
  # earlier in the file.
  # Here we turn the given point index triplets into an array of triangles
  # by creating triangles of points starting at v1/vt1/vn1.
  var faces = []
  var first = _split_point_idx(parts[1])
  var prev = _split_point_idx(parts[2])
  for i in range(3, len(parts)):
    var cur = _split_point_idx(parts[i])
    faces.append([first, prev, cur])
    prev = cur
  return faces

func parse(obj: String) -> Mesh:
  var verts = PoolVector3Array()
  var norms = PoolVector3Array()
  var textures = PoolVector2Array()
  var st = SurfaceTool.new()
  st.set_material(mat)
  st.begin(Mesh.PRIMITIVE_TRIANGLES)
  for line in obj.split(LINE_SEP, false):
    var parts = line.split(PART_SEP, false)
    match parts[0]:
      VERTEX:
        verts.push_back(Vector3(float(parts[1]), float(parts[2]), float(parts[3])))
      NORMAL:
        norms.push_back(Vector3(float(parts[1]), float(parts[2]), float(parts[3])))
      TEXTURE:
        textures.push_back(Vector2(float(parts[1]), float(parts[2])))
      FACE:
        for f in _parse_face(parts):
          # f[i][j] is the ith axis of the jth point in the face
          # Y and Z are swapped as OBJ exports Z-up by default (godot is Y-up)
          # We allow empty texture and normal coords if they're empty in OBJ (e.g. "1//")
          st.add_triangle_fan(
            [verts[f[0][0]], verts[f[2][0]], verts[f[1][0]]],
            [] if f[0][1] == -1 else [textures[f[0][1]], textures[f[2][1]], textures[f[1][1]]],
            PoolColorArray(),
            PoolVector2Array(),
            [] if f[0][2] == -1 else [norms[f[0][2]], norms[f[2][2]], norms[f[1][2]]],
            [])
  
  var mesh = Mesh.new()
  st.commit(mesh)
  return mesh
