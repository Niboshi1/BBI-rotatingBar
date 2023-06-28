import time
import struct

def write_arduino(arduino, target_position, light_strength, sig=0):
    arduino.write(struct.pack('>BBB',target_position,light_strength,sig))
