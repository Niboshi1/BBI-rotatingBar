import time

def write_arduino(arduino, x):
    arduino.write(bytes(str(x), 'utf-8'))
