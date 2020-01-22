
var buffer = PoolIntArray()
var pBuffer = PoolByteArray()
var bIdx = 0
var word = 0
var nibblesWritten = 0

func flushByte():
  print("%x" % word)
  pBuffer.push_back(word)
  print("%x" % pBuffer[0])
  nibblesWritten = 0
  word = 0

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
func EncodeVLE(value):
  # print("enc %x" % value)
  if value == 0 && nibblesWritten == 2:
    flushByte()
    return

  while value != 0:
    var nibble = value & 0x7 # lower 3 bits
    value >>= 3
    if value:
      nibble |= 0x8 # more to come
    word <<= 4
    word |= nibble
    # print("%x %x" % [nibble, word])
    nibblesWritten += 1
    if nibblesWritten == 2:
      flushByte()

# Decodes the variable length encoding scheme described above.
# 0010, 1101 --> 0000000000101010 = 42
# 42 = 0000000000101010 → 0010, 1101
# 010 --> 1010 -> a
# 101 --> 0101 -> 5
func DecodeVLE(buffer: PoolByteArray) -> int:
  var result = 0
  var nibble = 0
  var value16 = 0
  var bits = 29
  while true:
    if (!nibblesWritten):
      word = buffer[bIdx]
      bIdx += 1
      nibblesWritten = 2
    nibble = word & 0xf0
    result |= (nibble << 25) >> bits
    word <<= 4
    nibblesWritten -= 1
    bits -= 3
    if !(nibble & 0x80):
      break
  return result


func CompressRVL(input: PoolIntArray) -> PoolByteArray:
  pBuffer = PoolByteArray()
  nibblesWritten = 0
  var idx = 0
  while (idx < len(input)):
    var zeros = 0
    while idx != len(input) && input[idx] == 0:
      idx += 1
      zeros += 1
    EncodeVLE(zeros);
    
    var nonzeros = 0
    while idx+nonzeros != len(input) && input[idx+nonzeros] != 0:
      nonzeros += 1
    EncodeVLE(nonzeros);
    
    var i = 0
    while i < nonzeros:
      # TODO delta = current - previous, update prior frame
      var delta = input[idx]
      idx += 1
      
      var zigzag = (delta << 1) ^ (delta >> 31)
      EncodeVLE(zigzag)
      i += 1

  if nibblesWritten: # last few values
    pBuffer[idx] = word << 4 * (8 - nibblesWritten)
    idx += 1
  return pBuffer

func DecompressRVL(input: PoolByteArray, numVals: int) -> PoolIntArray:
  buffer = PoolIntArray()
  buffer.resize(numVals)
  pBuffer = input
  nibblesWritten = 0
  var current = 0
  var previous = 0
  bIdx = 0
  var idx = 0
  while idx < numVals:
    var zeros = DecodeVLE(pBuffer)
    print("Zeros %d" % zeros)
    for i in range(zeros):
      buffer[idx] = 0
      idx += 1
    var nonzeros = DecodeVLE(pBuffer)
    print("Nonzeros %d" % nonzeros)
    for i in range(nonzeros):
      var positive = DecodeVLE(pBuffer)
      var delta = (positive >> 1) ^ -(positive & 1)
      # TODO current = previous + delta
      buffer[idx] = delta
      idx += 1
      # previous = current
  return buffer
