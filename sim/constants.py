from sim.formulation import STATE_SPACE
import numpy as np

class ROBOT:
    # initial robot orientation parallel to x axis, center (0, 0)

    LENGTH = 12                     # [in]
    WIDTH = 18                      # [in]
    WHEEL_RADIUS = 6                # [in]
    AXLE_POS = 6                    # [in]
    WHEEL_SPACING = 18
    SERVO_SPEED = 2#13.6/2            # rad/s, 13.6 rad/s no load, nominal speed ~1/2

    # pos of middle sensor
    SENSOR_POS = np.array([AXLE_POS+3, -8])#-9.75])      # [in]
    SENSOR_LENGTH = 0.5             # [in]
    SENSOR_WIDTH = 0.5              # [in]
    SENSOR_SEP = 0.25               # [in]
    NUM_SENSORS = 5

    I_X = 0
    I_Y = 1
    I_THETA = 2
    I_SENSE = 0

    # sensor noise probability
    S_NOISE = 0.000

    # acutation noise probabilities
    TRANS_NOISE = 0
    ROT_NOISE = 0
    TRANS_MAG = 0.1
    ROT_MAG = 0.1

class ENVIRONMENT:
    LENGTH = 1.0    # [m]
    WIDTH = 1.0     # [m]

    # 1 pixel == 0.01 in
    IMAGE_SCALE = 1 / 0.01
    INPUT_SCALE = 1 / 1
    DISPLAY_SCALE = 1 / (1/8)
    INITIAL_ROBOT_STATE = [0.0 for k in STATE_SPACE]


class TEST:
    DT = 0.1  # [s]
    DISPLAY_RATE = 10 # every 10 timesteps

class DATA:
    TIMESTAMP = 'timestamp'
    WOODBOT = 'woodbot'
