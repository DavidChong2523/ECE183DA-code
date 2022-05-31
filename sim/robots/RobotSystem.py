from sim.constants import ROBOT, ENVIRONMENT
from sim.formulation import INPUT_SPACE, OUTPUT_SPACE


class RobotSystem:
    def __init__(self):
        """init with a specific initial state (optional) """
        self.state = ENVIRONMENT.INITIAL_ROBOT_STATE
        
        self.inpt = [0.0 for k in INPUT_SPACE]
        self.outpt = [0.0 for k in OUTPUT_SPACE]
        self._t_minus_1 = 0.0       # delta t may not be a constant
        self._DT = None   # flag if the robot is synchronous (block thread for delta_t)

    def drive(self, inpts, timestamp):
        """drive the robot to the next state
        :param inpts: left, right wheel velocities
        :return full state feedback"""
        return self.state

    def sense(self):
        """generate the sensor reading
        :return output"""
        return self.outpt
