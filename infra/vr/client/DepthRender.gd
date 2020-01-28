extends Spatial

onready var geom = $Geometry

# This doesn't actually get set, it's just a convenient way to stream in data
puppet var point_data setget point_data_set

var rvl = load("res://addons/rvl/main.gd").new()
const SCALE = 0.01
const W = 128
const H = 128
const SCALE_FACTOR = 1000.0

func _ready():
  self.set_network_master(1) # Server owns us

func point_data_set(data):
  if typeof(data) != TYPE_RAW_ARRAY:
    print("Got wrong type:", typeof(rvl.encoded))
    return
  rvl.Clear()
  rvl.encoded = data
  rvl.DecompressRVL(W*H)
  geom.clear()
  geom.begin(Mesh.PRIMITIVE_POINTS, null)
  var row = 0
  var col = 0
  for v in rvl.plain: #list of Vector3s
    geom.add_vertex(Vector3(row*SCALE, v / SCALE_FACTOR, col*SCALE))
    row += 1
    if row >= W:
      row = 0
      col += 1
    if col >= H:
      break
  geom.end()
  pass # Replace with function body.
