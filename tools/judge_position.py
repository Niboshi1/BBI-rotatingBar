def compare_results(target_angle, rat_position, window):
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
        return True
    else:
        return False