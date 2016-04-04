import numpy as np
import struct

x = 2.2
b8_array = np.array([x], dtype=np.float64)
print b8_array.tostring() + '\n'
b8 = b8_array.view(np.uint8)
extra_array = np.array([1,2,3], dtype=np.uint8)
array = [b8, extra_array]
num_ints = len(array)
byte_string = ""
for i in range(0, num_ints):
    byte_string += struct.pack('!B', array[i])