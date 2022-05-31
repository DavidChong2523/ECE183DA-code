import numpy as np
import cv2 as cv
from sim.constants import *

from sim.environment import LineEnv

def test_pid():
    readings = [1, 1, 1, 0, 1]
    def sensor_to_vel(reading_pos):
        # quadratic scale
        omega = 0*(reading_pos**2) + 1*reading_pos
        #omega = omega * np.sign(reading_pos)
        return omega

    values = [sensor_to_vel(i-ROBOT.NUM_SENSORS//2) for i, r in enumerate(readings) if r]
    
    if(len(values) == 0):
        curr_p = 0
    else:
        curr_p = sum(values) / len(values)
    print("readings:", readings, "values:", values, "p:", curr_p)

if __name__ == "__main__":
    test_pid()