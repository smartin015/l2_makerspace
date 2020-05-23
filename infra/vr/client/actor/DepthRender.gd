extends Spatial

var rvl = load("res://addons/rvl/main.gd").new()

# TODO properly handle projection from 2D to 3D via shader,
# using the camera matrix intrincic values.
# See rs2_deproject_pixel_to_point():
# https://github.com/IntelRealSense/librealsense/blob/master/include/librealsense2/rsutil.h#L67
const SCALE = 0.01
const SCALE_FACTOR = 0.0010000000474974513

onready var geom = $Geometry
var dims = [0,0]

func setup(width: int, height: int, topic: String):
  dims = [width, height]
  rvl.Init(width, height, 0)
  ROSBridge.ros_connect("0", 
    "sensor_msgs/CompressedImage", 
    self, "_point_data_set", 
    "depth_render_sub", 
    true) # Raw

func _point_data_set(data, id):
  if typeof(data) != TYPE_RAW_ARRAY:
    print("Got wrong type:", typeof(data))
    return
  if len(data) == 0:
    return
  # print("%d: %d %d %d" % [len(data), data[0], data[1], data[2]])
  # Second byte in the frame is whether this is a keyframe
  # If so, clear all past data
  rvl.encoded = data
  if !rvl.Decompress():
    print("Failed decompression")
  geom.clear()
  geom.begin(Mesh.PRIMITIVE_POINTS, null)
  var row = 0
  var col = 0
  for v in rvl.plain: #list of Vector3s
    geom.add_vertex(Vector3(row*SCALE, v*SCALE_FACTOR, col*SCALE))
    row += 1
    if row >= dims[0]:
      row = 0
      col += 1
    if col >= dims[1]:
      break
  geom.end()
