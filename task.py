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
import os

class BBI_Trainer(object):

    def __init__(self, config):
        # set config
        self.config = config

    def session_8way(self):
        # define Arduino
        arduino = serial.Serial(port='COM5', baudrate=115200, timeout=.1)
        arduino_mfb = serial.Serial(port='COM4', baudrate=115200, timeout=.1)
        time.sleep(3) # wait for Arduino to initialize

        # parameters for the light pattern
        w = np.uint32(self.config["img_w"])
        h = np.uint32(self.config["img_h"])
        func = np.uint32(self.config["func_n"])

        # parameters for LED light
        light_strength = self.config["maximum_brightness"] # np.random.randint(config["minimum_brightness"], config["maximum_brightness"])

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
        filename = os.path.join(self.config["save_dir"], 'behavioral_results' + time.strftime("%Y%m%d-%H%M%S") + '.csv')

        # start session
        start = time.time()
        trial_num = 0
        correct_trial_num = 0

        # connect to server
        if self.config["tcp_ip_connection"]:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            remote_ip = socket.gethostbyname(self.config["host"])
            s.connect((remote_ip, self.config["port"]))

        # open mapped memory
        mmap_file = mmap.mmap(-1, 4, "my_mapping", access=mmap.ACCESS_READ)
        msg = mmap_file.read(4)
        init_rat_position = float(struct.unpack('f', msg)[0])

        with open(filename, 'a') as f:
            write = csv.writer(f)
            write.writerow(headers)

            while True:
                # configure flash interval
                if flash_frame%self.config["flash_thresh"] == 0:
                    flash_OnOFF = True
                    flash_frame = 1
                else:
                    flash_OnOFF = False
                    flash_frame += 1

                # raise task difficulity after 20 correct trials
                if correct_trial_num == 20:
                    self.config["minimum_brightness"] = 1
                    self.config["maximum_brightness"] = 20

                # get angle from c++
                mmap_file = mmap.mmap(-1, 4, "my_mapping", access=mmap.ACCESS_READ)
                msg = mmap_file.read(4)
                rat_position = float(struct.unpack('f', msg)[0])

                # check if the position of rat is inside the window
                rat_reached_target = tools.compare_angles(target_angle, rat_position, self.config["window"])

                # check if the rat is taking the shortest path
                if self.config["task_shortest_path"]:
                    if shortest_path == "cw":
                        # wrong path is positive
                        if tools.get_angle_diff(init_rat_position, rat_position) > self.config["shortest_path_thresh"]:
                            task_failed = True
                    elif shortest_path == "ccw":
                        # wrong path is negative
                        if tools.get_angle_diff(init_rat_position, rat_position) < -1*self.config["shortest_path_thresh"]:
                            task_failed = True
                    else:
                        task_failed = False

                # display result
                img = self.draw_image(
                    target_angle, 
                    rat_position, 
                    init_rat_position,
                    rat_reached_target,
                    shortest_path,
                    )

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
                if self.config["tcp_ip_connection"]:
                    s.send(func)
                    s.send(w)
                    s.send(h)
                    s.send(tools.draw_bar(math.radians(float(target_angle)), w, h))

                # generate new target
                if rat_reached_target or elapsed_time > self.config["trial_length"] or task_failed:

                    # give reward if rat reached target
                    if rat_reached_target:
                        # increment correct trial number
                        correct_trial_num += 1

                        # change behavior depending on task_mode
                        if self.config["task_mode"] == "training":
                            reward_start = time.time()
                            tools.write_arduino(arduino, target_position, light_strength, 1)
                            arduino_mfb.write(bytes("1", 'utf-8'))

                            # give reward if rat is still inside the reward zone
                            while time.time()-reward_start < self.config["duration_after_reach"]: 
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
                                    if tools.compare_angles(target_angle, rat_position, self.config["window"]):
                                        tools.write_arduino(arduino, target_position, light_strength, 1)
                                        arduino_mfb.write(bytes("1", 'utf-8'))
                                        print(rat_position, round(target_angle%360), light_strength)

                        elif self.config["task_mode"] == "test":
                            # send signal to Arduino
                            tools.write_arduino(arduino, target_position, light_strength, 1)
                            arduino_mfb.write(bytes("1", 'utf-8'))
                            
                            print(rat_position, round(target_angle%360), light_strength)



                    # trial intervals
                    if rat_reached_target:
                        # shorter interval
                        tools.write_arduino(arduino, target_position, light_strength, 2)
                        #time.sleep(self.config["trial_interval"])
                    else:
                        #long interval
                        tools.write_arduino(arduino, target_position, light_strength, 3)
                        time.sleep(self.config["trial_interval"])


                    # Prepare for next trial===========================================

                    # Get initial rat position
                    mmap_file = mmap.mmap(-1, 4, "my_mapping", access=mmap.ACCESS_READ)
                    msg = mmap_file.read(4)
                    init_rat_position = float(struct.unpack('f', msg)[0])

                    # generate new target that is different from previous
                    prev_target_position = target_position
                    if self.config["led_selection_mode"] == "random":
                        while target_position == prev_target_position:
                            target_position = np.random.randint(0, 8)
                    if self.config["led_selection_mode"] == "adjacent":
                        target_position = target_position + np.random.choice([1,-1])
                        # fix if there is overlap [0, 1, 2, 3, 4, 5, 6, 7]
                        if target_position == -1:
                            target_position = 7
                        if target_position == 8:
                            target_position = 0
                    if self.config["led_selection_mode"] == "force_counter_rotation":
                        if rat_reached_target:
                            while target_position == prev_target_position or tools.calculate_shortest_path(init_rat_position, target_angle) == shortest_path:
                                target_position = np.random.randint(0, 8)
                                target_angle = target_position*360/8
                        else:
                            while target_position == prev_target_position or tools.calculate_shortest_path(init_rat_position, target_angle) != shortest_path:
                                target_position = np.random.randint(0, 8)
                                target_angle = target_position*360/8

                    target_angle = target_position*360/8
                    
                    # gnerate new led_brightness
                    light_strength = int(np.random.randint(self.config["minimum_brightness"], self.config["maximum_brightness"]))

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

                #time.sleep(config["update_speed"]/1000)

        # close mmap
        mmap_file.close()

        #close file
        f.close()

        # close tcp connection
        if self.config["tcp_ip_connection"]:
            s.close()

    def session_randwalk(self):
        # define Arduino
        arduino = serial.Serial(port='COM5', baudrate=115200, timeout=.1)
        arduino_mfb = serial.Serial(port='COM4', baudrate=115200, timeout=.1)
        time.sleep(3) # wait for Arduino to initialize

        # parameters for the light pattern
        w = np.uint32(self.config["img_w"])
        h = np.uint32(self.config["img_h"])
        func = np.uint32(self.config["func_n"])

        # parameters for LED light
        light_strength = self.config["maximum_brightness"] # np.random.randint(config["minimum_brightness"], config["maximum_brightness"])

        # initialize target
        n_leds = 188 # number of addresable LEDs on track
        target_position = 24 # 24 is equivalent to  45 degress on track
        target_angle = target_position/n_leds*360
        tools.write_arduino(arduino, target_position, light_strength, 0)

        # generate random walk path
        random_walk_path = tools.random_walk_filtered(init_position=target_position, step_n=70000, step=3)

        rat_reached_target = False
        time_within_target_win = time.time()
        thresh_time_reward = 1 # seconds
        cooldown = False
        thresh_cooldown = 1 # seconds
        cooldown_start = 0

        flash_OnOFF = True
        flash_frame = 1

        # define save file
        headers = ["time", "none", "rat_position", "none", "target_position", "reached_target", "brightness"]
        filename = os.path.join(self.config["save_dir"], 'behavioral_results' + time.strftime("%Y%m%d-%H%M%S") + '.csv')

        # start session
        start = time.time()
        walk_idx = 1

        # connect to server
        if self.config["tcp_ip_connection"]:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            remote_ip = socket.gethostbyname(self.config["host"])
            s.connect((remote_ip, self.config["port"]))

        # open mapped memory
        mmap_file = mmap.mmap(-1, 4, "my_mapping", access=mmap.ACCESS_READ)
        msg = mmap_file.read(4)
        init_rat_position = float(struct.unpack('f', msg)[0])

        with open(filename, 'a') as f:
            write = csv.writer(f)
            write.writerow(headers)

            while True:
                # configure flash interval
                if flash_frame%self.config["flash_thresh"] == 0:
                    flash_OnOFF = True
                    flash_frame = 1
                else:
                    flash_OnOFF = False
                    flash_frame += 1

                # get angle from c++
                mmap_file = mmap.mmap(-1, 4, "my_mapping", access=mmap.ACCESS_READ)
                msg = mmap_file.read(4)
                rat_position = float(struct.unpack('f', msg)[0])

                # check if the position of rat is inside the window
                rat_reached_target = tools.compare_angles(target_angle, rat_position, self.config["window"])

                # display result
                img = self.draw_image(
                    target_angle, 
                    rat_position, 
                    init_rat_position,
                    rat_reached_target,
                    None,
                    )

                cv2.imshow('animation', img)
                if cv2.waitKey(1) == ord('q'):
                    # press q to terminate the loop
                    cv2.destroyAllWindows()

                    # trun off lights on track
                    tools.write_arduino(arduino, 9, light_strength, 3)
                    break

                elapsed_time = time.time()-start
                write.writerow([elapsed_time, -1, rat_position, -1, round(target_position%360), rat_reached_target, light_strength])

                # send stimulus to server
                if self.config["tcp_ip_connection"]:
                    s.send(func)
                    s.send(w)
                    s.send(h)
                    s.send(tools.draw_bar(math.radians(float(target_angle)), w, h))

                # give reward
                if cooldown:
                    if time.time() - cooldown_start > thresh_cooldown:
                        cooldown = False
                    time_within_target_win = time.time()
                else:
                    if rat_reached_target:
                        # give reward if rat reached target for continuously over 1 seconds
                        if time.time()-time_within_target_win > thresh_time_reward:
                            # send signal to Arduino
                            arduino_mfb.write(bytes("1", 'utf-8'))
                            print(rat_position, round(target_angle%360), walk_idx, time.time())
                            cooldown_start = time.time()
                            cooldown = True
                            # reset
                            time_within_target_win = time.time()
                    # reset time if rat is outside the boundary
                    else:
                        time_within_target_win = time.time()
                #print(rat_position, round(target_angle%360), "     ", end='\r')

                # generate new target based on random walk
                target_position = int(random_walk_path[walk_idx])%n_leds
                target_angle = target_position/n_leds*360
                
                # gnerate new led_brightness
                light_strength = int(np.random.randint(self.config["minimum_brightness"], self.config["maximum_brightness"]))

                # send new target to arduino
                tools.write_arduino(arduino, target_position, light_strength, 0)
                time.sleep(1/100)
    
                # reset parameters
                rat_reached_target = False
                walk_idx += 1

                #time.sleep(config["update_speed"]/1000)

        # close mmap
        mmap_file.close()

        #close file
        f.close()

        # close tcp connection
        if self.config["tcp_ip_connection"]:
            s.close()


    def draw_image(
            self,
            target_angle, 
            rat_position, 
            init_rat_position,
            rat_reached_target,
            shortest_path=None
            ):
        
        # Load parameters
        sample_dim = self.config["sample_dim"]
        radius = int(sample_dim*0.3)
        line_thickness = 1

        # display result
        img = np.zeros((sample_dim, sample_dim, 3))
        x = int(sample_dim/2 + np.cos(math.radians(target_angle))*(radius))
        y = int(sample_dim/2 - np.sin(math.radians(target_angle))*(radius))

        x_rat = int(sample_dim/2 + np.cos(math.radians(rat_position))*(radius))
        y_rat = int(sample_dim/2 - np.sin(math.radians(rat_position))*(radius))

        cv2.ellipse(img, (int(sample_dim/2), int(sample_dim/2)), (radius, radius), 0, (target_angle+self.config["window"]/2)*-1, (target_angle-self.config["window"]/2)*-1, 255, -1)

        if shortest_path != None:
            # line indicating threshold for wrong trial
            if shortest_path == "cw":
                cw_ccw = 1
            elif shortest_path == "ccw":
                cw_ccw = -1
            else:
                cw_ccw = 0
            cv2.line(
                img, 
                (int(sample_dim/2), int(sample_dim/2)), 
                (
                    int(sample_dim/2 + np.cos(math.radians(init_rat_position+cw_ccw*self.config["shortest_path_thresh"]))*(radius)), 
                    int(sample_dim/2 - np.sin(math.radians(init_rat_position+cw_ccw*self.config["shortest_path_thresh"]))*(radius))
                    ), 
                (0, 0, 255), 
                thickness=line_thickness
                )

        # line indicating rat position
        cv2.line(img, (int(sample_dim/2), int(sample_dim/2)), (x_rat, y_rat), (255, 100, 0), thickness=line_thickness)

        # draw a circle when rat reaches target
        if rat_reached_target == True:
            cv2.circle(img, (int(sample_dim/10), int(sample_dim/10)), 5, (0, 0, 255), -1)

        return img