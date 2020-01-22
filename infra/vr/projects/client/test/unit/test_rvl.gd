extends "res://addons/gut/test.gd"

var rvl = load("res://addons/rvl/main.gd").new()

func test_vle_encode():
  rvl.EncodeVLE(42)
  var want = 0xa5
  assert_eq("%x" % rvl.pBuffer[0], "%x" % want, "variable length encoding")
 
func test_vle_decode():
  rvl.pBuffer = PoolByteArray([0xa5])
  var got = rvl.DecodeVLE(rvl.pBuffer, 0)
  var want = 42
  assert_eq("%x" % got, "%x" % want, "variable length decoding")

func test_rvl_compress():
  var data = PoolIntArray([1, 5, -5, 0xfffffe])
  var want = PoolByteArray([0x42,0xA1,0x91,0xCF,0x00,0xFF,0xFF]) # 16 bytes --> 7 bytes
  var got = rvl.CompressRVL(data)
  assert_eq(len(got), len(want), "RVL compress length")
  for i in range(len(got)):
    assert_eq(want[i], got[i], "RVL compression at index %d" % i)

func test_rvl_decompress():
  var data = PoolByteArray([0x42,0xA1,0x91,0xCF,0x00,0xFF,0xFF])
  var want = PoolIntArray([1, 5, -5, 0xfffffe])
  var got = rvl.DecompressRVL(data, len(want))
  assert_eq(len(got), len(want), "RVL compress length")
  for i in range(len(got)):
    assert_eq(want[i], got[i], "RVL compression at index %d" % i)

func test_rvl_encode_decode():
  # TODO encode and decode a bunch of examples 
  # e.g. 
  # []
  # [0]
  # [0xffff]
  # [0,0,0,0,1,1,2,2,0,0,0]
  pass
