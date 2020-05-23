extends "res://addons/gut/test.gd"

var rvl = load("res://addons/rvl/main.gd").new()

var K_HDR = [0, 1] # Channel 0, always keyframe
var RVL_TEST_DATA = [ # [plain, encoded, description]
  [[0], K_HDR+[0x10], "1 zero"],
  [[0, 0, 0], K_HDR+[0x30], "3 zeros"],
  [[-5], K_HDR+[0x01, 0x91], "-5 zigzags to 9. 0|000 0|001 1|001 0|001"],
  [[4], K_HDR+[0x01, 0x81], "0|000 0|001 1|000 0|001 --> 0x0, 1x 001000"],
  [[0, 1], K_HDR+[0x11, 0x20], "0 then 1"],
  [[1, 0], K_HDR+[0x01, 0x21, 0x00], "1 then 0; 0000"], 
  [[1, 1, 2, 3, 5], K_HDR+[0x05, 0x22, 0x46, 0xa1], "10B -> 4B"],
  [[1, 5, -5], K_HDR+[0x03, 0x2a, 0x19, 0x10], "0|000 0|011 0|010 1|001 0|001 1|001 --> 0x0, 3xnonzero, 1 (zig 0x02), 5 (zig 0xa)"],
  [[0,0,0,0,1,0,0,1,0,0], K_HDR+[0x41,0x22,0x12,0x20], "sparse ones"],
  [[0,0,-1,0,0,1,0,0,-1,0,0], K_HDR+[0x21,0x12,0x12,0x21,0x12,0x00], "negatives: 44B -> 6B"],
  # 5000 = 0x1388, double to 10000 for zig-zag
  # 10000 = 0x2710 -(x2b)> 0010 0111 0001 0000 
  # -(3)> 0 010 011 100 010 000 -(flip&cb)> 1|000 1|010 1|100 1|011 0|010
  # --> 8 A C B 2 -> 0x8acb2
  [[5000], K_HDR+[0x01, 0x8a, 0xcb, 0x20], "large value"], 
  [[5000, 5000], K_HDR+[0x02, 0x8a, 0xcb, 0x28, 0xac, 0xb2], "large value x2"], 
]

func byte_str(a):
  var output = PoolStringArray()
  for i in a:
    output.push_back("%02x" % i)
  return "[%s]" % [output.join(", ")]

func assert_bytes_eq(got, want):
  if len(got) != len(want):
    assert_eq(len(got), len(want), "list length mismatch: got %s want %s" % [byte_str(got), byte_str(want)])
    return
  var output = PoolStringArray()
  var fail = false
  for i in range(len(got)):
    var s = "0x%02x" % got[i]
    var w = want[i]
    if "%02x" % got[i] != "%02x" % w:
      s += " (want 0x%02x)" % w
      fail = true
    output.push_back(s)
  assert_false(fail, "Mismatch: [%s]" % [output.join(", ")])

func test_rvl_compress():
  for v in RVL_TEST_DATA:
    print("Case: %s" % v[2])
    rvl.Init(1,len(v[0]),0,0)
    rvl.plain = PoolIntArray(v[0])
    rvl.Compress()
    assert_bytes_eq(rvl.encoded, v[1])

func test_rvl_decompress():
  for v in RVL_TEST_DATA:
    print("Case: %s" % v[2])
    rvl.Init(1,len(v[0]),0,0)
    rvl.encoded = PoolByteArray(v[1])
    rvl.Decompress()
    assert_bytes_eq(rvl.plain, v[0])
#
#func test_rvl_keyframe_compress():
#  rvl.Init(1,1,0,1) # Keyframe every other frame
#  var enc4 = [0x01, 0x81]
#  rvl.plain = [4]
#  rvl.Compress()
#  # Keyframe on first frame
#  assert_bytes_eq(rvl.encoded, [0, 1] + enc4)
#
#  rvl.Compress()
#  # Zero delta from last frame
#  assert_bytes_eq(rvl.encoded, [0, 0, 0x10])
#
#  rvl.Compress()
#  # Keyframe again
#  assert_bytes_eq(rvl.encoded, [0, 1] + enc4)
#
#func test_rvl_keyframe_decompress():
#  rvl.Init(1,1,0) 
#  var enc4 = [0x01, 0x81]
#  rvl.encoded = [0, 1] + enc4
#  rvl.Decompress()
#  assert_bytes_eq(rvl.plain, [4])
#
#  # Delta should add
#  rvl.encoded = [0, 0] + enc4
#  rvl.Decompress()
#  assert_bytes_eq(rvl.plain, [8])
#
#  # Keyframe should clear
#  rvl.encoded = [0, 1] + enc4
#  rvl.Decompress()
#  assert_bytes_eq(rvl.plain, [4])
