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
  # A face is a vertex, texture, and normal, each defined by indexes into the set of vertices
  # earlier in the file.
  # Turn N points defining a face into an array of triangles
  # by selecting triplets of points fanning out from the first set of points
  # f v1/vt1/vn1 v2/vt2/vn2 v3/vt3/vn3... v#/vt#/vn#
  var faces = []
  var first = _split_point_idx(parts[1])
  var prev = _split_point_idx(parts[2])
  for i in range(3, len(parts)) :
    var cur = _split_point_idx(parts[i])
    faces.append([
      first[0], prev[0], cur[0], # Vertex
      first[1], prev[1], cur[1], # Texture
      first[2], prev[2], cur[2], # Normal
    ])
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
          # Y and Z are swapped as OBJ exports Z-up by default (godot is Y-up)
          # We allow empty texture and normal coords if they're empty in OBJ (e.g. "1//")
          st.add_triangle_fan(
            [verts[f[0]], verts[f[2]], verts[f[1]]],
            [] if f[3] == -1 else [textures[f[3]], textures[f[5]], textures[f[4]]],
            PoolColorArray(),
            PoolVector2Array(),
            [] if f[6] == -1 else [norms[f[6]], norms[f[8]], norms[f[7]]],
            [])
  
  var mesh = Mesh.new()
  st.commit(mesh)
  return mesh
