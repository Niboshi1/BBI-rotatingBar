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
import csv
import time

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
target_position = 1
target_angle = target_position*360/8
tools.write_arduino(arduino, str(target_position))
print("new_position", target_position)

window = 30 # in degrees
update_speed = 100 # in ms
rat_reached_target = False
flash_OnOFF = True
flash_thresh = 5
flash_frame = 1

# define save file
headers = ["time", "rat_position", "target_position", "reached_target"]
filename = 'behavioral_results' + time.strftime("%Y%m%d-%H%M%S") + '.csv'
start = time.time()
with open(filename, 'a') as f:
    write = csv.writer(f)
    write.writerow(headers)

# connect to server
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
remote_ip = socket.gethostbyname(host)
s.connect((remote_ip, port))

# define save file
headers = ["time", "rat_position", "target_position", "reached_target"]
filename = 'behavioral_results' + time.strftime("%Y%m%d-%H%M%S") + '.csv'
start = time.time()

with open(filename, 'a') as f:
    write = csv.writer(f)
    write.writerow(headers)
    
    while True:
        # configure flash interval
        if flash_frame%flash_thresh == 0:
            flash_OnOFF = True
            flash_frame = 1
        else:
            flash_OnOFF = False
            flash_frame += 1

        # get angle from c++
        mmap_file = mmap.mmap(-1, 4, "my_mapping", access=mmap.ACCESS_READ)
        msg = mmap_file.read(4)
        rat_position = float(struct.unpack('f', msg)[0])
        if rat_position < 0:
            rat_position = 360 + rat_position

        # convert rat_position to an alge assuming beggining of the window is 0
        fixed_rat_position = rat_position - (target_angle - window/2)
        if fixed_rat_position < 0:
            fixed_rat_position += 360
        elif fixed_rat_position >= 360:
            fixed_rat_position -= 360

        # check if the position of rat is inside the window
        if fixed_rat_position < window:
            rat_reached_target = True

            # send signal to Arduino
            tools.write_arduino(arduino, "s")
            print(rat_position, round(target_angle%360))

        # display result
        img = np.zeros((sample_dim, sample_dim, 3))
        x = int(sample_dim/2 + np.cos(math.radians(target_angle))*(radius))
        y = int(sample_dim/2 - np.sin(math.radians(target_angle))*(radius))

        x_rat = int(sample_dim/2 + np.cos(math.radians(rat_position))*(radius))
        y_rat = int(sample_dim/2 - np.sin(math.radians(rat_position))*(radius))

        cv2.ellipse(img, (int(sample_dim/2), int(sample_dim/2)), (radius, radius), 0, (target_angle+window/2)*-1, (target_angle-window/2)*-1, 255, -1)
        if flash_OnOFF == True:
            cv2.line(img, (int(sample_dim/2), int(sample_dim/2)), (x, y), (255, 255, 255), thickness=line_thickness)
        cv2.line(img, (int(sample_dim/2), int(sample_dim/2)), (x_rat, y_rat), (255, 100, 0), thickness=line_thickness)

        if rat_reached_target == True:
            cv2.circle(img, (int(sample_dim/10), int(sample_dim/10)), 5, (0, 0, 255), -1)

        cv2.imshow('animation', img)
        if cv2.waitKey(1) == ord('q'):
            # press q to terminate the loop
            cv2.destroyAllWindows()
            break

        print(rat_position, round(target_angle%360), flash_OnOFF, "     ", end='\r')
        
        elapsed_time = time.time()-start
        write.writerow([elapsed_time, rat_position, round(target_angle%360), rat_reached_target])

        # generate new target
        if rat_reached_target:
            # generate new target
            target_position = np.random.randint(0, 7)
            target_angle = target_position*360/8
            tools.write_arduino(arduino, str(target_position))
            
            # reset goal
            rat_reached_target = False

        time.sleep(update_speed/1000)
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
        s.send(tools.draw_bar(math.radians(float(target_angle)), img_w, img_h, flash_OnOFF))
        
        time.sleep(update_speed/1000)

# close TCP connection
mmap_file.close()
s.close()
#close file
f.close()