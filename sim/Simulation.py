import time
import logging
from sim.constants import TEST
from sim.formulation import INPUT_SPACE, STATE_SPACE, OUTPUT_SPACE, assert_space

class Sim:
    def __init__(self, use_noise=False, suppress_info=False):
        """
        :param suppress_info: no log outputs to console
        """
        self._use_noise = use_noise
        self.suppress_info = suppress_info
        self._input_system = None
        self._robots = []
        self._DT = None


    def set_input(self, input_system):
        """set InputSystem
        :param input_system: input to be used (child of InputSystem)"""
        self._input_system = input_system

    def add_robot(self, robot, outputs):
        """Add a robot to simulate and specify output forms
        :param robot: robot to simulate (child of RobotSystem)
        :param outputs: tuple of outputs (child of OutputSystem)
        """
        self._robots.append([robot, outputs])
        if robot._DT is not None:
            self._DT = robot._DT

    def run(self, max_duration):
        """Run robots with the given settings for max_duration seconds
        :param max_duration: the maximum duration of test
        """
        t = 0
        if self._input_system is None:
            raise ImportError
        
        loop = not max_duration
        while (loop) or (t < max_duration and not self._input_system.is_done()) :
            inpt = self._input_system.get_inputs(timestamp=t)
            assert_space(inpt, INPUT_SPACE)

            for robot, outputs in self._robots:
                state = robot.drive(inpt, t)
                assert_space(state, STATE_SPACE)

                outpt = robot.sense()
                assert_space(outpt, OUTPUT_SPACE)
                for out in outputs:
                    out.process(state, inpt, outpt, t)

                if not self.suppress_info:
                    logging.info("t: {}, inpt: {}, state: {}, outpt: {}".format(t, inpt, state, outpt))

                

            t = self._step_forward(t)

        self.make_outputs()

    def _step_forward(self, t):
        if self._DT is None:
            t += TEST.DT
            time.sleep(TEST.DT)
            #time.sleep(0.05)
        else:
            t = self._DT(t)
        return t

    def make_outputs(self):
        for robot, outputs in self._robots:
            for out in outputs:
                out.make_output()

