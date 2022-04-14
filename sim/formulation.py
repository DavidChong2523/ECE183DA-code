"""TODO: update STATE_SPACE and OUTPUT_SPACE"""
INPUT_SPACE = dict(
    motor_l = float,
    motor_r = float,
)

STATE_SPACE = dict(
    x = float,
    y = float,
    theta = float,
    theta_dot = float,
    sensor_x = float,
    sensor_y = float
)

OUTPUT_SPACE = dict(
    lidar_f = float,
    lidar_r = float,
    mag_x = float,
    mag_y = float,
    gyro_z = float,
    )

def assert_space(data, space):
    assert(len(data) == len(space))
    assert(all(type(x) == y for x, y in zip(data, space.values())))
