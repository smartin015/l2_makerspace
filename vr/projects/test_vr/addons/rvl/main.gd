
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
  print("enc %x" % value)
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
    print("%x %x" % [nibble, word])
    nibblesWritten += 1
    if nibblesWritten == 2: # output word
      flushByte()

func DecodeVLE(buffer: PoolIntArray, bPtr: int) -> int:
  var nibble = 0
  var value = 0
  var bits = 29
  while (nibble & 0x80000000):  # TODO this was a do while
    if (!nibblesWritten):
      # word = *pBuffer++; # load word
      nibblesWritten = 8
    nibble = word & 0xf0000000
    value |= (nibble << 1) >> bits
    word <<= 4
    nibblesWritten -= 1
    bits -= 3
  return value

"""
func CompressRVL(short* input, char* output, int numPixels) --> int:
  buffer = pBuffer = (int*)output
  nibblesWritten = 0
  short *end = input + numPixels
  short previous = 0
  while (input != end):
    int zeros = 0, nonzeros = 0
    for (; (input != end) && !*input; input++, zeros++):
      EncodeVLE(zeros); # number of zeros
    for (short* p = input; (p != end) && *p++; nonzeros++):
      EncodeVLE(nonzeros); # number of nonzeros
    for (int i = 0; i < nonzeros; i++):
      short current = *input++
      int delta = current - previous
      int positive = (delta << 1) ^ (delta >> 31)
      EncodeVLE(positive); # nonzero value
      previous = current
  if nibblesWritten: # last few values
    *pBuffer++ = word << 4 * (8 - nibblesWritten)
  return int((char*)pBuffer - (char*)buffer); # num bytes


func DecompressRVL(input: PoolIntArray, output: PoolByteArray, numPixels: int):
  buffer = PoolIntArray()
  pBuffer = PoolINtArray()
  nibblesWritten = 0
  var current, previous = 0
  var numPixelsToDecode = numPixels
  while (numPixelsToDecode):
    int zeros = DecodeVLE(); # number of zeros
    numPixelsToDecode -= zeros
    for (; zeros; zeros--):
      *output++ = 0
      int nonzeros = DecodeVLE(); # number of nonzeros
      numPixelsToDecode -= nonzeros
    for (; nonzeros; nonzeros--):
    int positive = DecodeVLE(); # nonzero value
    int delta = (positive >> 1) ^ -(positive & 1)
    current = previous + delta
    *output++ = current
    previous = current

"""
