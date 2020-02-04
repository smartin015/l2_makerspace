import unittest
import rvl

class TestVLE(unittest.TestCase):
    VLE_TESTS = [
      [42, [0xa5]], 
      [1, [0x10]], 
      # 300 = 0x012c --> 0001 0010 1100 --> 000 100 101 100 --> 0100 1101 1100 --> 4 D C --> 0xcd40
      [300, [0xcd, 0x40]], 
      [5280, [0x8c,0xaa,0x10]],
      [65535, [0xff,0xff,0xf1]],
    ]
    def test_vle_encode(self):
      for v in self.VLE_TESTS:
        rvl.Clear()
        rvl.EncodeVLE(v[0])
        rvl.Flush()
        self.assertEqual(len(rvl.encoded), len(v[1]), "VLE length for test %d" % v[0])
        for i in range(len(v[1])):
          self.assertEqual("%x" % rvl.encoded[i], "%x" % v[1][i], "VLE encoding at index %d for test %d" % (i, v[0]))

    def test_vle_decode(self):
      for v in self.VLE_TESTS:
        rvl.Clear()
        rvl.encoded = bytearray(v[1])
        got = rvl.DecodeVLE()
        self.assertEqual("%x" % v[0], "%x" % got, "decode VLE for test %d" % v[0])

class TestRVL(unittest.TestCase):
    RVL_TESTS = [
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

    def test_rvl_compress(self):
      for v in self.RVL_TESTS:
        rvl.Clear()
        rvl.plain = list(v[0])
        got = rvl.CompressRVL()
        self.assertEqual(len(rvl.encoded), len(v[1]), "RVL compress length for test")
        for i in range(len(v[1])):
          self.assertEqual("%02x" % rvl.encoded[i], "%02x" % v[1][i], "RVL compression at index %d for test %s" % (i, v[0]))

    def test_rvl_decompress(self):
      for v in self.RVL_TESTS:
        rvl.Clear()
        rvl.encoded = bytearray(v[1])
        got = rvl.DecompressRVL(len(v[0]))
        self.assertEqual(len(rvl.plain), len(v[0]), "RVL compress length for test")
        for i in range(len(v[0])):
          self.assertEqual(rvl.plain[i], v[0][i], "RVL compression at index %d for test %s" % (i, v[0]))


if __name__ == "__main__":
    unittest.main()
