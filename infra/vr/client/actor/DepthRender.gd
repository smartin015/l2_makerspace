extends Spatial

var rvl = load("res://addons/rvl/main.gd").new()

# TODO properly handle projection from 2D to 3D via shader,
# using the camera matrix intrincic values.
# See rs2_deproject_pixel_to_point():
# https://github.com/IntelRealSense/librealsense/blob/master/include/librealsense2/rsutil.h#L67
const SCALE = 0.01
const FLT_EPSILON = 0.0000001

onready var geom = $Geometry
var stride = 1
var width = 0
var height = 0
var depth_scale = 0.001
var ppx
var ppy
var fx
var fy
var coeffs

func setup(topic: String):
  ROSBridge.ros_connect("0", 
    "sensor_msgs/CompressedImage", 
    self, "_point_data_set", 
    "depth_render_sub", 
    true) # Raw

func _handle_settings_packet(data):
  var p = JSON.parse(data.subarray(2,-1).get_string_from_utf8())
  if p.error == OK:
    print("Received camera depth information: %s" % p.result)
    stride = p.result.get('stride', 1)
    width = int(p.result.get('width', 0))
    height = int(p.result.get('height', 0))
    depth_scale = p.result.get('depth_scale', depth_scale)
    print("%d %d %f" % [width, height, depth_scale])
    ppx = p.result['ppx']
    ppy = p.result['ppy']
    fx = p.result['fx']
    fy = p.result['fy']
    coeffs = p.result['coeffs']
    rvl.Init(width, height, 0)

# Given pixel coordinates and depth in an image with no distortion 
# or inverse distortion coefficients, compute the corresponding 
# point in 3D space relative to the same camera
func rs2_deproject_pixel_to_point(x, y, depth) -> Vector3:
  x = (x - ppx) / fx
  y = (y - ppy) / fy
  
  # Assume RS2_DISTORTION_KANNALA_BRANDT4
  # See https://github.com/hiroMTB/ofxRealsense2/pull/4/files
  var rd = sqrt(x*x + y*y);
  if rd < FLT_EPSILON:
    rd = FLT_EPSILON
  var theta = rd;
  var theta2 = rd*rd;
  for i in range(4):
    var f = theta*(1 + theta2*(coeffs[0] + theta2*(coeffs[1] + theta2*(coeffs[2] + theta2*coeffs[3])))) - rd;
    if abs(f) < FLT_EPSILON:
      break
    var df = 1 + theta2*(3 * coeffs[0] + theta2*(5 * coeffs[1] + theta2*(7 * coeffs[2] + 9 * theta2*coeffs[3])));
    theta -= f / df;
    theta2 = theta*theta;
  var r = tan(theta);
  x *= r / rd;
  y *= r / rd;
  
  return Vector3(depth * x, depth, depth * y)

func _point_data_set(data, _id):
  if typeof(data) != TYPE_RAW_ARRAY:
    print("Got wrong type:", typeof(data))
    return
  if len(data) == 0:
    return
  if data[1] == rvl.SETTINGS_PACKET_ID and width == 0 and height == 0:
    _handle_settings_packet(data)
    return
        
  if width == 0 or height == 0:
    return # Not yet configured

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
    if col >= width/stride:
      col = 0
      row += 1
    if row >= height/stride:
      break
    if v != 0:
      # Only show "nonzero distance" pixels
      # Y axis is inverted (image is y-down, godot is y-up)
      geom.add_vertex(rs2_deproject_pixel_to_point(col*stride, height-(row*stride), v*depth_scale))
    col += 1
    
  geom.end()
