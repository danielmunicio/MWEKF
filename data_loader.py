'''
For loading binary steering data from the SD card
'''

import struct
import matplotlib.pyplot as plt
fmt = "<LH" # need endian-ness indicator to eliminate padding
angles = []
times = []
with open("/media/reid/F4F3-7EE1/log3.bin", "rb") as f:
    while True:
        try: data = struct.unpack(fmt, f.read(struct.calcsize(fmt)))
        except: break # if we can't `f.read` any more
        angles.append(data[1])
        times.append(data[0])

plt.plot(x=times, y=angles)
plt.show()