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


# define Arduino
arduino = serial.Serial(port='COM5', baudrate=115200, timeout=.1)
time.sleep(3) # wait for Arduino to initialize

# choose mode "ranodm" or "adjacent"
# "random" : randomely select next target
# "adjacent" : select adjacent location
mode = "random"

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

# parameters for LED light
maximum_brightness = 255
minimum_brightness = 200
light_strength = maximum_brightness # np.random.randint(minimum_brightness, maximum_brightness)

# parameters for target
prev_target_position = -1
target_position = 1
target_angle = target_position*360/8
tools.write_arduino(arduino, target_position, light_strength, 0)

window = 30 # in degrees
update_speed = 100 # in ms
rat_reached_target = False
flash_OnOFF = True
flash_thresh = 5
flash_frame = 1

# parameters for trial
trial_length = 30*60 # seconds
trial_interval = 3 # seconds

# define save file
headers = ["time", "trial", "rat_position", "target_position_from",  "target_position_to", "reached_target"]
filename = 'behavioral_results' + time.strftime("%Y%m%d-%H%M%S") + '.csv'

# start session
start = time.time()
trial_num = 0

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

        print(rat_position, round(target_angle%360), "     ", end='\r')
        
        elapsed_time = time.time()-start
        write.writerow([elapsed_time, trial_num, rat_position, round(prev_target_position%360), round(target_position%360), rat_reached_target])

        # generate new target
        if rat_reached_target or elapsed_time > trial_length:

            # give reward if rat reached target
            if rat_reached_target:
                # send signal to Arduino
                tools.write_arduino(arduino, target_position, light_strength, 1)
                print(rat_position, round(target_angle%360), light_strength)


            # generate new target that is different from previous
            prev_target_position = target_position
            if mode == "random":
                while target_position == prev_target_position:
                    target_position = np.random.randint(0, 8)
            if mode == "adjacent":
                target_position = target_position + np.random.choice([1,-1])
                # fix if there is overlap [0, 1, 2, 3, 4, 5, 6, 7]
                if target_position == -1:
                    target_position = 7
                if target_position == 8:
                    target_position = 0

            target_angle = target_position*360/8

            # gnerate new led_brightness
            light_strength = int(np.random.randint(minimum_brightness, maximum_brightness))

            # write target to arduino
            tools.write_arduino(arduino, target_position, light_strength, 2)
            if rat_reached_target:
                # shorter interval
                time.sleep(trial_interval)
            else:
                #long interval
                time.sleep(trial_interval*2)
            tools.write_arduino(arduino, target_position, light_strength, 0)
            
            # reset goal
            rat_reached_target = False

            # next trial 
            trial_num += 1
            start = time.time()

        time.sleep(update_speed/1000)

# close TCP connection
mmap_file.close()

#close file
f.close()