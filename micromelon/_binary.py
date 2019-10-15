def bytesToIntArray(b, bytesPerInt, signed=True, endianness='little'):
  if len(b) % bytesPerInt != 0:
    raise Exception('Wrong number of bytes for conversion')
  nums = [0] * int((len(b) / bytesPerInt))
  for i in range(0, len(b), bytesPerInt):
    nums[int(i / bytesPerInt)] = int.from_bytes(b[i:i+bytesPerInt], byteorder=endianness, signed=signed)
  return nums

def intArrayToBytes(nums, bytesPerInt, signed=True, endianness='little'):
  b = []
  for i in range(len(nums)):
    b = b + list((nums[i]).to_bytes(bytesPerInt, byteorder=endianness, signed=signed))
  return b

def bytesToAsciiString(b):
  return ''.join(list(map(chr, b)))

def stringToBytes(s, encoding='ascii'):
  return list(bytes(s, encoding=encoding))
