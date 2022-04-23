"""TODO: update STATE_SPACE and OUTPUT_SPACE"""
INPUT_SPACE = dict(
    motor_l = float,
    motor_r = float,
)

STATE_SPACE = dict(
    x = float,
    y = float,
    theta = float,
    sx = float,
    sy = float
)

OUTPUT_SPACE = dict(
    line_sensor = [float]    
)

def assert_space(data, space):
    assert(len(data) == len(space))
    # assert(all(type(x) == y for x, y in zip(data, space.values())))
