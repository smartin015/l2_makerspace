extends Spatial

var rvl = load("res://addons/rvl/main.gd").new()

# TODO properly handle projection from 2D to 3D via shader,
# using the camera matrix intrincic values.
# See rs2_deproject_pixel_to_point():
# https://github.com/IntelRealSense/librealsense/blob/master/include/librealsense2/rsutil.h#L67
const SCALE = 0.1
const FLT_EPSILON = 0.0000001

onready var ind = $Indicator
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
var success = null
var arr = []
var verts = PoolVector3Array()
var vertsUpdated = false

func setup(topic: String):
  ROSBridge.ros_connect("0", 
    "sensor_msgs/CompressedImage", 
    self, "_point_data_received", 
    "depth_render_sub", 
    true) # Raw

func _ready():
  ind.material_override = SpatialMaterial.new()

func _process(_delta):
  if success != null:
    if success:
      ind.material_override.albedo_color = Color(0, 1, 0)
    else:
      ind.material_override.albedo_color = Color(1, 0, 0)
    success = null
  
  if vertsUpdated:
    geom.mesh.surface_remove(0)
    arr[Mesh.ARRAY_VERTEX] = verts
    geom.mesh.add_surface_from_arrays(Mesh.PRIMITIVE_POINTS, arr)
    vertsUpdated = false

func _init_mesh():
  arr.resize(Mesh.ARRAY_MAX)
  var l = (width/stride)*(height/stride)
  verts.resize(l)
  var uvs = PoolVector2Array()
  uvs.resize(l)
  var normals = PoolVector3Array()
  normals.resize(l)
  var indices = PoolIntArray()
  indices.resize(l)
  for r in range(height/stride):
    for c in range(width/stride):
      var i = int(r*(width/stride) + c)
      indices.set(i, i)
      verts.set(i, Vector3.ZERO)

  # Assign arrays to mesh array.
  arr[Mesh.ARRAY_VERTEX] = verts
  arr[Mesh.ARRAY_TEX_UV] = uvs
  arr[Mesh.ARRAY_NORMAL] = normals
  arr[Mesh.ARRAY_INDEX] = indices

  # Create mesh surface from mesh array.
  geom.mesh.add_surface_from_arrays(Mesh.PRIMITIVE_POINTS, arr)

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
    rvl.Init((width/stride)*(height/stride), 0)
    _init_mesh()

# Given pixel coordinates and depth in an image with no distortion 
# or inverse distortion coefficients, compute the corresponding 
# point in 3D space relative to the same camera
func rs2_deproject_pixel_to_point(x, y, depth) -> Vector3:
  x = (x - ppx) / fx
  y = (y - ppy) / fy
  
  # Assume RS2_DISTORTION_KANNALA_BRANDT4
  # See https://github.com/hiroMTB/ofxRealsense2/pull/4/files
  #  var rd = sqrt(x*x + y*y);
  #  if rd < FLT_EPSILON:
  #    rd = FLT_EPSILON
  #  var theta = rd;
  #  var theta2 = rd*rd;
  #  for i in range(4):
  #    var f = theta*(1 + theta2*(coeffs[0] + theta2*(coeffs[1] + theta2*(coeffs[2] + theta2*coeffs[3])))) - rd;
  #    if abs(f) < FLT_EPSILON:
  #      break
  #    var df = 1 + theta2*(3 * coeffs[0] + theta2*(5 * coeffs[1] + theta2*(7 * coeffs[2] + 9 * theta2*coeffs[3])));
  #    theta -= f / df;
  #    theta2 = theta*theta;
  #  var r = tan(theta);
  #  x *= r / rd;
  #  y *= r / rd;
  return Vector3(depth * x, depth, depth * y)

func _point_data_received(data, _id):
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

  rvl.encoded = data
  success = rvl.Decompress()
  for i in range(len(rvl.plain)):
    verts[i] = rs2_deproject_pixel_to_point(
      (i % int(width/stride)) * stride, 
      height - (int(i / (width/stride)) * stride), 
      rvl.plain[i] * depth_scale)
  vertsUpdated = true
#    verts[i] = Vector3(
#        (x-(width/stride/2))*SCALE, 
#        rvl.plain[i]*depth_scale, 
#        ((height/stride/2)-y)*SCALE)

    
