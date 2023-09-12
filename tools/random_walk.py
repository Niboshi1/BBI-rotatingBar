import numpy as np
import matplotlib.pyplot as plt
from scipy import signal

def butter_lowpass(lowcut, fs, order=4):
    '''バターワースローパスフィルタを設計する関数
    '''
    nyq = 0.5 * fs
    low = lowcut / nyq
    b, a = signal.butter(order, low, btype='low')
    return b, a


def butter_lowpass_filter(x, lowcut, fs, order=4):
    '''データにローパスフィルタをかける関数
    '''
    b, a = butter_lowpass(lowcut, fs, order=order)
    y = signal.filtfilt(b, a, x)
    return y

def random_walk_filtered(init_position, step_n=70000, step=3):
    # Define parameters for the walk
    step_set = [int(-1*step), 0, step]

    # Simulate steps in 1D
    steps = np.random.choice(a=step_set, size=step_n)
    path = np.concatenate([[init_position], steps]).cumsum(0)
    path_filt = butter_lowpass_filter(path, 3, 1000)

    return path_filt