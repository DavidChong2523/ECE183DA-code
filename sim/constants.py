from sim.formulation import STATE_SPACE

class ROBOT:
    LENGTH = 12                    # [m]
    WIDTH = 18                     # [m]
    WHEEL_RADIUS = 6                # [m]
    FRONT_LIDAR_POS = [0.055, 0.0]    # [m]
    RIGHT_LIDAR_POS = [0.047, 0.015]    # [m]
    SERVO_SPEED = 13.6/2            # rad/s, 13.6 rad/s no load, nominal speed ~1/2

    SENSOR_POS = [8, 8.5]
    SENSOR_LENGTH = 2
    SENSOR_WIDTH = 2

class ENVIRONMENT:
    LENGTH = 1.0    # [m]
    WIDTH = 1.0     # [m]
    INITIAL_ROBOT_STATE = [0.0 for k in STATE_SPACE]


class TEST:
    DT = 0.1  # [s]


class DATA:
    TIMESTAMP = 'timestamp'
    WOODBOT = 'woodbot'
