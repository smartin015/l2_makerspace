extends "res://addons/gut/test.gd"

var rvl = load("res://addons/rvl/main.gd").new()

var VLE_TESTS = [
  [42, [0xa5]], 
  [1, [0x10]], 
  # 300 = 0x012c --> 0001 0010 1100 --> 000 100 101 100 --> 0100 1101 1100 --> 4 D C --> 0xcd40
  [300, [0xcd, 0x40]], 
  [5280, [0x8c,0xaa,0x10]],
  [65535, [0xff,0xff,0xf1]],
]

func test_vle_encode():
  for v in VLE_TESTS:
    rvl.Clear()
    rvl.EncodeVLE(v[0])
    rvl.Flush()
    assert_eq(len(rvl.encoded), len(v[1]), "VLE length for test %d" % v[0])
    for i in range(len(v[1])):
      assert_eq("%x" % rvl.encoded[i], "%x" % v[1][i], "VLE encoding at index %d for test %d" % [i, v[0]])

func test_vle_decode():
  for v in VLE_TESTS:
    rvl.Clear()
    rvl.encoded = PoolByteArray(v[1])
    var got = rvl.DecodeVLE()
    assert_eq("%x" % v[0], "%x" % got, "decode VLE for test %d" % v[0])

var RVL_TESTS = [
  [[0], [0x10]], # "1 zero"
  [[0, 0, 0], [0x30]], # "3 zeros"
  [[-5], [0x01, 0x91]],  # -5 zigzags to 9. 0|000 0|001 1|001 0|001
  [[4], [0x01, 0x81]], # 0|000 0|001 1|000 0|001 --> 0x0, 1x 001000
  [[0, 1], [0x11, 0x20]],
  [[1, 0], [0x01, 0x21, 0x00]], # 0000
  [[1, 1, 2, 3, 5], [0x05, 0x22, 0x46, 0xa1]],  # 10B -> 4B
  [[1, 5, -5], [0x03, 0x2a, 0x19, 0x10]], # 0|000 0|011 0|010 1|001 0|001 1|001 --> 0x0, 3xnonzero, 1 (zig 0x02), 5 (zig 0xa),  
  [[0,0,0,0,1,0,0,1,0,0], [0x41,0x22,0x12,0x20]],
  [[0,0,-1,0,0,1,0,0,-1,0,0], [0x21,0x12,0x12,0x21,0x12,0x00]], # 44B -> 6B
]

func test_rvl_compress():
  for v in RVL_TESTS:
    rvl.Clear()
    rvl.plain = PoolIntArray(v[0])
    var got = rvl.CompressRVL()
    assert_eq(len(rvl.encoded), len(v[1]), "RVL compress length for test")
    for i in range(len(v[1])):
      assert_eq("%02x" % rvl.encoded[i], "%02x" % v[1][i], "RVL compression at index %d for test %s" % [i, v[0]])

func test_rvl_decompress():
  for v in RVL_TESTS:
    rvl.Clear()
    rvl.encoded = PoolByteArray(v[1])
    var got = rvl.DecompressRVL(len(v[0]))
    assert_eq(len(rvl.plain), len(v[0]), "RVL compress length for test")
    for i in range(len(v[0])):
      assert_eq(rvl.plain[i], v[0][i], "RVL compression at index %d for test %s" % [i, v[0]])
  #assert_eq(len(got), len(want), "RVL compress length")
  #for i in range(len(got)):
  #  assert_eq(want[i], got[i], "RVL compression at index %d" % i)
