from sim.robots.RobotSystem import *
from sim.formulation import *

class NOPModel(RobotSystem):
    def drive(self, inpt, timestamp):
        assert_space(inpt, INPUT_SPACE)
        return [0.0 for k in STATE_SPACE]

    def sense(self):
        return [0.0 for k in OUTPUT_SPACE]
