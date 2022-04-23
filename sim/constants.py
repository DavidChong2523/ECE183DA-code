from sim.formulation import STATE_SPACE
import numpy as np

class ROBOT:
    # initial robot orientation parallel to x axis, center (0, 0)

    LENGTH = 12                     # [in]
    WIDTH = 18                      # [in]
    WHEEL_RADIUS = 6                # [in]
    SERVO_SPEED = 13.6/2            # rad/s, 13.6 rad/s no load, nominal speed ~1/2

    # pos of middle sensor
    SENSOR_POS = np.array([8.5, -10.75])      # [in]
    SENSOR_LENGTH = 0.5             # [in]
    SENSOR_WIDTH = 0.5              # [in]
    SENSOR_SEP = 0.25               # [in]
    NUM_SENSORS = 5

    I_X = 0
    I_Y = 1
    I_THETA = 2
    I_SENSE = 0

class ENVIRONMENT:
    LENGTH = 1.0    # [m]
    WIDTH = 1.0     # [m]

    # 1 pixel == 0.01 in
    IMAGE_SCALE = 1 / 0.01
    INPUT_SCALE = 1 / 1
    DISPLAY_SCALE = 1 / 1
    INITIAL_ROBOT_STATE = [0.0 for k in STATE_SPACE]


class TEST:
    DT = 0.1  # [s]


class DATA:
    TIMESTAMP = 'timestamp'
    WOODBOT = 'woodbot'
