
plain = []
encoded = bytearray()
nibs = 0
byte = 0
decodeIdx = 0

def Clear():
  global plain, encoded, nibs, byte, decodeIdx
  plain = []
  encoded = bytearray()
  nibs = 0
  byte = 0
  decodeIdx = 0

def Flush():
  global plain, encoded, nibs, byte, decodeIdx
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
# 42 = 0000000000101010 -> 0010, 1101
# 010 --> 1010 -> a
# 101 --> 0101 -> 5
# Precondition: value must be positive or zero
def EncodeVLE(value: int):
  global plain, encoded, nibs, byte, decodeIdx
  if value == 0 and nibs == 2:
    Flush()

  ct = True
  while ct:
    nibble = value & 0x7 # lower 3 bits
    value >>= 3
    if value:
      nibble |= 0x8 # more to come
    byte = (byte << 4) | nibble
    nibs += 1
    if nibs == 2:
      Flush()
    ct = (value != 0)

# Decodes the variable length encoding scheme described above.
# 0010, 1101 --> 0000000000101010 = 42
# 42 = 0000000000101010 → 0010, 1101
# 010 --> 1010 -> a
# 101 --> 0101 -> 5
# Returns [result value, nibbles consumed]
def DecodeVLE():
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


def CompressRVL(chan):
  global plain, encoded, nibs, byte, decodeIdx
  encoded.append(chan)
  idx = 0
  while (idx < len(plain)):
    zeros = 0
    while idx < len(plain) and plain[idx] == 0:
      idx += 1
      zeros += 1
    # print("Encoded %d x 0" % zeros)
    EncodeVLE(zeros);

    nonzeros = 0
    while idx+nonzeros < len(plain) and plain[idx+nonzeros] != 0:
      nonzeros += 1
    # print("Encoded %d ! 0" % nonzeros)
    EncodeVLE(nonzeros);

    i = 0
    while i < nonzeros:
      # TODO delta = current - previous, update prior frame
      delta = plain[idx]
      idx += 1

      zigzag = (delta << 1) ^ (delta >> 31)
      # print("Encoded %02x zigzag (%d)" % [zigzag, delta])
      EncodeVLE(zigzag)
      i += 1
  
  #if nibs: # last few values
  #  encoded[idx] = byte << 4 * (8 - nibs)
  #  idx += 1
  Flush()

def DecompressRVL(numVals: int):
  global plain, encoded, nibs, byte, decodeIdx
  plain = [0] * numVals
  idx = 0
  while idx < numVals and decodeIdx < len(encoded):
    zeros = DecodeVLE()
    # print("Zeros %d" % zeros)
    for i in range(zeros):
      plain[idx] = 0
      idx += 1
    nonzeros = DecodeVLE()
    # print("Nonzeros %d" % nonzeros)
    for i in range(nonzeros):
      zigzag = DecodeVLE()
      delta = (zigzag >> 1) ^ -(zigzag & 1)
      # print("Got %02x zigzag (%d)" % [zigzag, delta])
      # TODO current = previous + delta
      plain[idx] = delta
      idx += 1
      # previous = current
