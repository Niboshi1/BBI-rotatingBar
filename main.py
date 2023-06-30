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

from settings import OPTIONS

# define Arduino
arduino = serial.Serial(port='COM5', baudrate=115200, timeout=.1)
time.sleep(3) # wait for Arduino to initialize

# create start image
sample_dim = OPTIONS["sample_dim"]
radius = int(sample_dim*0.3)
line_thickness = 1

# parameters for the light pattern
w = np.uint32(OPTIONS["img_w"])
h = np.uint32(OPTIONS["img_h"])
func = np.uint32(OPTIONS["func_n"])

# parameters for LED light
light_strength = OPTIONS["maximum_brightness"] # np.random.randint(OPTIONS["minimum_brightness"], OPTIONS["maximum_brightness"])

# initialize target
prev_target_position = -1
target_position = 1
target_angle = target_position*360/8
tools.write_arduino(arduino, target_position, light_strength, 0)

# initial parameters for shortest path task
shortest_path = None
task_failed = False

rat_reached_target = False
flash_OnOFF = True
flash_frame = 1

# define save file
headers = ["time", "trial", "rat_position", "target_position_from",  "target_position_to", "reached_target", "brightness"]
filename = 'behavioral_results' + time.strftime("%Y%m%d-%H%M%S") + '.csv'

# start session
start = time.time()
trial_num = 0
correct_trial_num = 0

# connect to server
if OPTIONS["tcp_ip_connection"]:
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    remote_ip = socket.gethostbyname(OPTIONS["host"])
    s.connect((remote_ip, OPTIONS["port"]))

# open mapped memory
mmap_file = mmap.mmap(-1, 4, "my_mapping", access=mmap.ACCESS_READ)
msg = mmap_file.read(4)
init_rat_position = float(struct.unpack('f', msg)[0])

with open(filename, 'a') as f:
    write = csv.writer(f)
    write.writerow(headers)

    while True:
        # configure flash interval
        if flash_frame%OPTIONS["flash_thresh"] == 0:
            flash_OnOFF = True
            flash_frame = 1
        else:
            flash_OnOFF = False
            flash_frame += 1

        # raise task difficulity after 20 correct trials
        if correct_trial_num == 20:
            OPTIONS["minimum_brightness"] = 1
            OPTIONS["maximum_brightness"] = 20

        # get angle from c++
        mmap_file = mmap.mmap(-1, 4, "my_mapping", access=mmap.ACCESS_READ)
        msg = mmap_file.read(4)
        rat_position = float(struct.unpack('f', msg)[0])

        # check if the position of rat is inside the window
        rat_reached_target = tools.compare_angles(target_angle, rat_position, OPTIONS["window"])

        # check if the rat is taking the shortest path
        if OPTIONS["task_shortest_path"]:
            if shortest_path == "cw":
                # wrong path is positive
                if tools.get_angle_diff(init_rat_position, rat_position) > OPTIONS["shortest_path_thresh"]:
                    task_failed = True
            elif shortest_path == "ccw":
                # wrong path is negative
                if tools.get_angle_diff(init_rat_position, rat_position) < -1*OPTIONS["shortest_path_thresh"]:
                    task_failed = True
            else:
                task_failed = False

        # display result
        img = np.zeros((sample_dim, sample_dim, 3))
        x = int(sample_dim/2 + np.cos(math.radians(target_angle))*(radius))
        y = int(sample_dim/2 - np.sin(math.radians(target_angle))*(radius))

        x_rat = int(sample_dim/2 + np.cos(math.radians(rat_position))*(radius))
        y_rat = int(sample_dim/2 - np.sin(math.radians(rat_position))*(radius))

        cv2.ellipse(img, (int(sample_dim/2), int(sample_dim/2)), (radius, radius), 0, (target_angle+OPTIONS["window"]/2)*-1, (target_angle-OPTIONS["window"]/2)*-1, 255, -1)
        if flash_OnOFF == True:
            cv2.line(img, (int(sample_dim/2), int(sample_dim/2)), (x, y), (255, 255, 255), thickness=line_thickness)
        cv2.line(img, (int(sample_dim/2), int(sample_dim/2)), (x_rat, y_rat), (255, 100, 0), thickness=line_thickness)

        # draw a circle when rat reaches target
        if rat_reached_target == True:
            cv2.circle(img, (int(sample_dim/10), int(sample_dim/10)), 5, (0, 0, 255), -1)

        cv2.imshow('animation', img)
        if cv2.waitKey(1) == ord('q'):
            # press q to terminate the loop
            cv2.destroyAllWindows()

            # trun off lights on track
            tools.write_arduino(arduino, 9, light_strength, 3)
            break

        print(rat_position, round(target_angle%360), "     ", end='\r')
        
        elapsed_time = time.time()-start
        write.writerow([elapsed_time, trial_num, rat_position, round(prev_target_position%360), round(target_position%360), rat_reached_target, light_strength])

        # send stimulus to server
        if OPTIONS["tcp_ip_connection"]:
            s.send(func)
            s.send(w)
            s.send(h)
            s.send(tools.draw_bar(math.radians(float(target_angle)), w, h))

        # generate new target
        if rat_reached_target or elapsed_time > OPTIONS["trial_length"] or task_failed:

            # give reward if rat reached target
            if rat_reached_target:
                # increment correct trial number
                correct_trial_num += 1

                # change behavior depending on task_mode
                if OPTIONS["task_mode"] == "training":
                    reward_start = time.time()
                    tools.write_arduino(arduino, target_position, light_strength, 1)

                    # give reward if rat is still inside the reward zone
                    while time.time()-reward_start < OPTIONS["duration_after_reach"]: 
                        # check if arduino is ready to recieve signal
                        arduino_state = arduino.readline().decode().rstrip()
                        if len(arduino_state) != 0:
                            arduino_ready = arduino_state[-1]
                        
                        if arduino_ready == "1":
                            # get angle from c++
                            mmap_file = mmap.mmap(-1, 4, "my_mapping", access=mmap.ACCESS_READ)
                            msg = mmap_file.read(4)
                            rat_position = float(struct.unpack('f', msg)[0])

                            # check if the rat is still in the target zone
                            if tools.compare_angles(target_angle, rat_position, OPTIONS["window"]):
                                tools.write_arduino(arduino, target_position, light_strength, 1)
                                print(rat_position, round(target_angle%360), light_strength)

                elif OPTIONS["task_mode"] == "test":
                    # send signal to Arduino
                    tools.write_arduino(arduino, target_position, light_strength, 1)
                    print(rat_position, round(target_angle%360), light_strength)



            # trial intervals
            tools.write_arduino(arduino, target_position, light_strength, 2)
            if rat_reached_target:
                # shorter interval
                time.sleep(OPTIONS["trial_interval"])
            else:
                #long interval
                time.sleep(OPTIONS["trial_interval"]*2)


            # Prepare for next trial===========================================

            # Get initial rat position
            mmap_file = mmap.mmap(-1, 4, "my_mapping", access=mmap.ACCESS_READ)
            msg = mmap_file.read(4)
            init_rat_position = float(struct.unpack('f', msg)[0])

            # generate new target that is different from previous
            prev_target_position = target_position
            if OPTIONS["led_selection_mode"] == "random":
                while target_position == prev_target_position:
                    target_position = np.random.randint(0, 8)
            if OPTIONS["led_selection_mode"] == "adjacent":
                target_position = target_position + np.random.choice([1,-1])
                # fix if there is overlap [0, 1, 2, 3, 4, 5, 6, 7]
                if target_position == -1:
                    target_position = 7
                if target_position == 8:
                    target_position = 0
            target_angle = target_position*360/8
            
            # gnerate new led_brightness
            light_strength = int(np.random.randint(OPTIONS["minimum_brightness"], OPTIONS["maximum_brightness"]))

            # send new target to arduino
            tools.write_arduino(arduino, target_position, light_strength, 0)

            # calculate if the shortest path is cw or ccw
            shortest_path = tools.calculate_shortest_path(init_rat_position, target_angle)
            print("Shortest path is :", shortest_path)
                
            # reset parameters
            task_failed = False
            rat_reached_target = False

            # next trial 
            trial_num += 1
            start = time.time()

        #time.sleep(OPTIONS["update_speed"]/1000)

# close mmap
mmap_file.close()

#close file
f.close()

# close tcp connection
if OPTIONS["tcp_ip_connection"]:
    s.close()