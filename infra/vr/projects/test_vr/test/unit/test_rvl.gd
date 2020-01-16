extends "res://addons/gut/test.gd"

var rvl = load("res://addons/rvl/main.gd").new()

func test_vle_encode():
  rvl.EncodeVLE(42)
  var want = 0xa5
  assert_eq("%x" % rvl.pBuffer[0], "%x" % want, "variable length encoding")
  # rv1.DecodeVLE(venc, 0, vout, 0)
 
func test_vle_decode():
  rv1.DecodeVLE()
  rv1.pBuffer = PoolByteArray([0xa5])
  var want = 42
  assert_eq("%x" % rv1.dBuffer[0], "%x" % want, "variable length decoding")
  
func test_rvl_encode_decode():
  # TODO encode and decode a bunch of examples 
  # e.g. 
  # []
  # [0]
  # [0xffff]
  # [0,0,0,0,1,1,2,2,0,0,0]
