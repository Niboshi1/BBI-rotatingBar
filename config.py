import numpy as np

OPTIONS = {
    # choose led_selection_mode "ranodm" or "adjacent"
    # "random" : randomely select next target
    # "adjacent" : select adjacent location
    # "force_counter_rotation" : select backward target position 
    "led_selection_mode" : "random",

    # choose task_mode "training" or "test"
    # "training" : LED will be on for 3 seconds even after the rat reached target
    # "test" : LED will trun off after the rat reached target
    "task_mode" : "training",
    # if task_mode is "training", change how long rats can be in the area
    "duration_after_reach" : 3, # in seconds

    "random_walk":True,
    "random_walk_blind":True,

    # task to force the rat to take the shortest path
    "task_shortest_path" : True,
    "shortest_path_thresh" : 30, # degrees to allow the rat to take the wrong path

    # whether to send signal to OASYS
    "tcp_ip_connection" : False,
    # define TCP server address
    "host" : "localhost",
    "port" : 2222,

    # create start image
    "sample_dim" : 200,

    # parameters for the light pattern
    "img_w" : 820,
    "img_h" : 552,
    "func_n" : 1,

    # parameters for LED light
    "maximum_brightness" : 255,
    "minimum_brightness" : 20,

    # parameters for target
    "window" : 30, # in degrees, threshold around the target
    "update_speed" : 100, # in ms, interval to send tcp/ip to OASYS
    "flash_thresh" : 5, # loop numebr, how often photostimulation should be turned on

    # parameters for trial
    "trial_length" : 5*60, # seconds, maximum length before target is updated
    "trial_interval" : 2, # seconds, interval between trials
}