import numpy as np
import pytest
import logging

from sim.robots import NOPModel, KinematicsModel
from sim.formulation import *
from sim.constants import *

@pytest.mark.parametrize("model", [NOPModel, KinematicsModel])
def test_model(model):
    """Evaluate a RobotSystem model and raise exception if invalid
    :param model: class (inherited from RobotSystem) to be tested
    """

    t = model()

    outpt1 = t.sense()
    assert_space(outpt1, OUTPUT_SPACE)

    outpt2 = t.sense()
    assert outpt1 == outpt2

    # TODO: Add additional tests to ensure module correctness
    XIND = 0
    YIND = 1
    THETAIND = 2
    THETADOTIND = 3

    LIDFIND = 0
    LIDRIND = 1
    GYROIND = 4
    def checks(t, out):
        # check model stays within bounds of environment
        assert(np.abs(t.state[XIND]) < 0.5)
        assert(np.abs(t.state[YIND]) < 0.5)

        # check model angle within [0, 2pi)
        assert(np.abs(t.state[THETAIND]) < 2*np.pi)

        # check model lidar never larger than 1
        # check model lidar always positive
        assert(0 <= out[LIDFIND] <= 1)
        assert(0 <= out[LIDRIND] <= 1)

        # check gyro z equals angular velocity
        assert(t.state[THETADOTIND] == out[GYROIND])

    dt = TEST.DT
    init_orients = [0, np.pi/2, np.pi, 3*np.pi/2]
    for o in init_orients:
        t = model()
        t.state = [0.0, 0.0, o, 0.0]
        init_out = [i for i in t.sense()]
        for i in np.linspace(0, 10, 1000):
            i = float(i)
            t.drive((1.0, 1.0), i)
            out = t.sense()
            checks(t, out)
        # check the robot moved
        final_out = t.sense()
        assert(final_out != init_out)

if __name__ == "__main__":
    test_model()




