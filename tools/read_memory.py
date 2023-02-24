import mmap
import struct

try:
    mmap_file = mmap.mmap(-1, 4, "my_mapping", access=mmap.ACCESS_READ)
    msg = mmap_file.read(4)
    print(float(struct.unpack('f', msg)[0]))
except Exception as e:
    print("An error occurred:", e)
finally:
    mmap_file.close()