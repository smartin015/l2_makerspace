extends "res://addons/gut/test.gd"

var rvl = load("res://addons/rvl/main.gd").new()

var VLE_TESTS = [
  [42, [0xa5]], 
  [1, [0x10]], 
  # 300 = 0x012c --> 0001 0010 1100 --> 000 100 101 100 --> 0100 1101 1100 --> 4 D C --> 0xcd40
  [300, [0xcd, 0x40]], 
  # 5280 = 0x14A0 -(x2b)> 0001 0100 1010 0000 
  # -(3)> 001 010 010 100 000 -(flip&cb)> 1|000 1|100 1|010 1|010 0|001 -> 8 C A A 1
  [5280, [0x8c,0xaa,0x10]],
  [65535, [0xff,0xff,0xf1]],
  # 10000 = 0x2710 -(x2b)> 0010 0111 0001 0000 
  # -(3)> 0 010 011 100 010 000 -(flip&cb)> 1|000 1|010 1|100 1|011 0|010
  # --> 8 A C B 2 -> 0x8acb20
  [10000, [0x8a,0xcb,0x20]],
]

func test_vle_encode():
  for v in VLE_TESTS:
    rvl.Init(1,1,0)
    rvl._encodeVLE(v[0])
    rvl._flush()
    assert_eq(len(rvl.encoded), len(v[1]), "VLE length for test %d" % v[0])
    for i in range(len(v[1])):
      assert_eq("%x" % rvl.encoded[i], "%x" % v[1][i], "VLE encoding at index %d for test %d" % [i, v[0]])

func test_vle_encode_multi():
  rvl.Init(1,2,0)
  rvl._encodeVLE(10000)
  rvl._encodeVLE(10000)
  rvl._flush()
  var want = [0x8a, 0xcb, 0x28, 0xac, 0xb2]
  assert_eq(len(rvl.encoded), len(want), "VLE length")
  for i in range(len(want)):
    assert_eq("%x" % rvl.encoded[i], "%x" % want[i], "VLE encoding at index %d" % [i])

func test_vle_decode():
  for v in VLE_TESTS:
    rvl.Init(1,1,0)
    rvl.encoded = PoolByteArray(v[1])
    var got = rvl._decodeVLE()
    assert_eq("%x" % v[0], "%x" % got, "decode VLE for test %d" % v[0])
