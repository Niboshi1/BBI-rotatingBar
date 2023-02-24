import mmap
from struct import unpack
import socket
import numpy as np
import time
import struct
import tools
import serial
import math
import cv2

# define TCP server address
host = "localhost"
port = 2222

# define Arduino
arduino = serial.Serial(port='COM3', baudrate=115200, timeout=.1)

# create start image
sample_dim = 200
radius = int(sample_dim*0.3)
line_thickness = 1

# parameters for the light pattern
img_w = 820
img_h = 552
w = np.uint32(img_w)
h = np.uint32(img_h)
func = np.uint32(1)

# parameters for target
target_angle = 0
window = 80 # in degrees
update_speed = 100 # in ms
step_degree = 72 # in degrees/sec
step = step_degree/(1000/update_speed) # step to update speed
stim_thresh = 1000 # duration that the rat must stay in the window
duration = 0 # in ms
rat_inside_window = False


# connect to server
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
remote_ip = socket.gethostbyname(host)
s.connect((remote_ip, port))

print("thresh: ", stim_thresh, -2)

while True:
    # get angle from c++
    mmap_file = mmap.mmap(-1, 4, "my_mapping", access=mmap.ACCESS_READ)
    msg = mmap_file.read(4)
    rat_position = float(struct.unpack('f', msg)[0])
    target_angle = tools.angle_step(target_angle, step)%360

    # convert rat_position to an alge assuming beggining of the window is 0
    if target_angle - window/2 > 0:
        _target_angle = target_angle - window/2
    else:
        _target_angle = target_angle - window/2 + 360

    if rat_position > _target_angle:
        fixed_rat_position = rat_position - _target_angle
    else:
        fixed_rat_position = rat_position - _target_angle + 360

    # check if the position of rat is inside the window
    if fixed_rat_position < window:
        if rat_inside_window == True:
            if duration<stim_thresh:
                duration += update_speed
            else:
                # send signal to Arduino
                tools.write_arduino(arduino, 1)
                print(rat_position, round(target_angle%360), duration)
                # reset
                duration = 0
        elif rat_inside_window == False:
            rat_inside_window = True
    else:
        rat_inside_window = False
        duration = 0

    # display result
    img = np.zeros((sample_dim, sample_dim, 3))
    x = int(sample_dim/2 + np.cos(math.radians(target_angle))*(radius))
    y = int(sample_dim/2 - np.sin(math.radians(target_angle))*(radius))

    x_rat = int(sample_dim/2 + np.cos(math.radians(rat_position))*(radius))
    y_rat = int(sample_dim/2 - np.sin(math.radians(rat_position))*(radius))

    cv2.ellipse(img, (int(sample_dim/2), int(sample_dim/2)), (radius, radius), 0, (target_angle+window/2)*-1, (target_angle-window/2)*-1, 255, -1)
    cv2.line(img, (int(sample_dim/2), int(sample_dim/2)), (x, y), (255, 255, 255), thickness=line_thickness)
    cv2.line(img, (int(sample_dim/2), int(sample_dim/2)), (x_rat, y_rat), (255, 100, 0), thickness=line_thickness)

    if rat_inside_window == True:
        cv2.circle(img, (int(sample_dim/10), int(sample_dim/10)), 5, (0, 0, 255), -1)

    cv2.imshow('animation', img)
    if cv2.waitKey(1) == ord('q'):
        # press q to terminate the loop
        cv2.destroyAllWindows()
        break

    print(rat_position, round(target_angle%360), duration, "     ", end='\r')

    # Send image to server
    # Below, the variable can be changed to either "1" or "2", which affects how the image is interpreted by PolyScan2.
    # If "1", the uploaded image will fit within the EWA of the Polygon
    # (e.g. entire image will be seen in EWA)
    # If "2", the uploaded image will be truncated, so only the portion of the image within the EWA will be projected.
    # (e.g. image fits within camera window, but only the portion of the image seen in EWA will be projected)

    # send binary pattern to server ( 1bit/pixel )
    
    s.send(func)
    s.send(w)
    s.send(h)
    s.send(tools.draw_bar(math.radians(float(target_angle)), img_w, img_h))
    
    time.sleep(update_speed/1000)

# close TCP connection
mmap_file.close()
s.close()