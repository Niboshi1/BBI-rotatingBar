import numpy as np

def get_angle_diff(angle_from, angle_to):
    # fix both angles to be inside 0-360
    if angle_from < 0:
        angle_from = 360 + angle_from

    if angle_to < 0:
        angle_to = 360 + angle_to

    # calculate difference between angle_from and angle_to
    # ex1. angle_to(130) - angle_from(90) = 40
    # ex2. angle_to(90) - angle_from(130) = -40

    angle_diff = angle_to - angle_from
    if angle_diff < -180:
        angle_diff += 360
    elif angle_diff >= 180:
        angle_diff -= 360

    return angle_diff

def compare_angles(target_angle, rat_position, window):
    angle_diff = np.abs(get_angle_diff(target_angle, rat_position))

    # check if the position of rat is inside the window
    if angle_diff < window/2:
        return True
    else:
        return False
    
def calculate_shortest_path(init_rat_position, target_angle):
    # calculate if the shortest path is cw or ccw
    if np.abs(get_angle_diff(init_rat_position, target_angle)) > 150:
        shortest_path = "both"
    else:
        if get_angle_diff(init_rat_position, target_angle) > 0:
            shortest_path = "ccw"
        elif get_angle_diff(init_rat_position, target_angle) < 0:
            shortest_path = "cw"

    return shortest_path