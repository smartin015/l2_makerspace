# RVL depth camera compression impl
# See https://www.microsoft.com/en-us/research/uploads/prod/2018/09/p100-wilson.pdf
# for the original research paper.
# 
# This implementation uses the same zigzag + variable-length encoding
# scheme, but with a custom packet:
# 
# - 1 byte channel ID
# - 1 byte keyframe status (0 == use delta, nonzero == treat as absolute values)
# - n bytes data
#
# NOTE: This file can be converted into a 
# python3-compatible module when build_py.sh
# is invoked.
# 
# For lines where conversion isn't easy,
# use "" to prefix a python  line


init = []
plain = []
encoded = bytearray()
prev = []
keyframe_pd = 0
last_keyframe = 0
chan = 0




nibs = 0
byte = 0
decodeIdx = 0

def Init(w: int, h: int, channel: int, keyframe_period: int):
  global plain, encoded, nibs, byte, decodeIdx, init, keyframe_pd, chan
  init = bytearray(int(w*h))



  
  encoded = bytearray()

  _clear_prev()
  _clear_plain()
  nibs = 0
  byte = 0
  decodeIdx = 0
  keyframe_pd = keyframe_period
  chan = channel

def _clear_prev():
  global prev, init
  prev = list(init)



def _clear_plain():
  global plain, init
  plain = list(init)



def _flush():
  global nibs, byte
  if nibs == 0:
    return
  elif nibs == 2:
    # print("flushed x%02x" % byte)
    encoded.append(byte)

  elif nibs == 1:
    encoded.append((byte << 4) & 0xff)

  nibs = 0
  byte = 0

# The number of zeros and non-zeros are positive integers
# written to the compressed bitstream with a base-8 variable
# length encoding scheme: starting with the least significant
# bits, the integer is broken into successive groups of three bits
# until all set bits are consumed. A fourth continuation bit is
# added to each group of three indicating whether it is the last
# group. For example, the 16 bit value 42 is encoded as 8 bits:
# 42 = 0000000000101010 → 0010, 1101
# 010 --> 1010 -> a
# 101 --> 0101 -> 5
# Precondition: value must be positive or zero
def _encodeVLE(value: int):
  global plain, encoded, nibs, byte, decodeIdx
  if value == 0 and nibs == 2:
    _flush()

  ct = True
  while ct:
    nibble = value & 0x7 # lower 3 bits
    value >>= 3
    if value:
      nibble |= 0x8 # more to come
    byte = (byte << 4) | nibble
    nibs += 1
    if nibs == 2:
      _flush()
    ct = (value != 0)

# Decodes the variable length encoding scheme described above.
# 0010, 1101 --> 0000000000101010 = 42
# 42 = 0000000000101010 → 0010, 1101
# 010 --> 1010 -> a
# 101 --> 0101 -> 5
# Returns [result value, nibbles consumed]
def _decodeVLE():
  global plain, encoded, nibs, byte, decodeIdx
  result = 0
  bits = 29
  while True:
    if nibs <= 0:
      if decodeIdx >= len(encoded):
        return result
      byte = encoded[decodeIdx]
      decodeIdx += 1
      nibs = 2
    nibble = byte & 0x70
    ct = byte & 0x80
    result |= (nibble << 25) >> bits
    # print("byte %02x nibble %02x result %02x ctd %s" % [byte, nibble, result, ct])
    byte = (byte << 4) & 0xff
    # print("byte now %02x" % byte)
    nibs -= 1
    bits -= 3
    if not ct:
      break
  return result

def Compress():
  global plain, encoded, nibs, byte, decodeIdx, chan, keyframe_pd, last_keyframe, prev
  encoded = bytearray()
  encoded.append(chan)


  if last_keyframe == 0:
    encoded.append(1)

    _clear_prev()
  else:
    encoded.append(0)

    
  last_keyframe += 1
  if last_keyframe > keyframe_pd:
    last_keyframe = 0
  
  idx = 0
  while (idx < len(plain)):
    zeros = 0
    while idx < len(plain) and (plain[idx] - prev[idx]) == 0:
      idx += 1
      zeros += 1
    # print("Encoded %d x 0" % zeros)
    _encodeVLE(zeros);

    nonzeros = 0
    while idx+nonzeros < len(plain) and (plain[idx+nonzeros] - prev[idx+nonzeros]) != 0:
      nonzeros += 1
    # print("Encoded %d ! 0" % nonzeros)
    _encodeVLE(nonzeros);

    i = 0
    while i < nonzeros:
      delta = plain[idx] - prev[idx]
      idx += 1

      zigzag = (delta << 1) ^ (delta >> 31)
      # print("Encoded %02x zigzag (%d)" % [zigzag, delta])
      _encodeVLE(zigzag)
      i += 1
  
  #if nibs: # last few values
  #  encoded[idx] = byte << 4 * (8 - nibs)
  #  idx += 1
  _flush()
  # Update our cache
  prev = plain

def Decompress():
  global plain, encoded, nibs, byte, decodeIdx
  if encoded[1] != 0:
    _clear_plain()
  
  plainIdx = 0 
  decodeIdx = 2 # Header length
  while plainIdx < len(init) and decodeIdx < len(encoded):
    zeros = _decodeVLE()
    plainIdx += zeros
    nonzeros = _decodeVLE()
    for i in range(nonzeros):
      if plainIdx >= len(plain):
        return False # Decompression failed (overrun)
      zigzag = _decodeVLE()
      delta = (zigzag >> 1) ^ -(zigzag & 1)
      # print("Got %02x zigzag (%d)" % [zigzag, delta])
      plain[plainIdx] += delta
      plainIdx += 1
  return True
