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
    rvl.Init(32,32,0,0)
    rvl._encodeVLE(v[0])
    rvl._flush()
    assert_eq(len(rvl.encoded), len(v[1]), "VLE length for test %d" % v[0])
    for i in range(len(v[1])):
      assert_eq("%x" % rvl.encoded[i], "%x" % v[1][i], "VLE encoding at index %d for test %d" % [i, v[0]])

func test_vle_decode():
  for v in VLE_TESTS:
    rvl.Init(32,32,0,0)
    rvl.encoded = PoolByteArray(v[1])
    var got = rvl._decodeVLE()
    assert_eq("%x" % v[0], "%x" % got, "decode VLE for test %d" % v[0])
