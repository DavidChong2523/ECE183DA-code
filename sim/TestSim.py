import time
import logging
from sim.constants import TEST
from sim.formulation import INPUT_SPACE, STATE_SPACE, OUTPUT_SPACE, assert_space
import numpy as np

class TestSim:
    def __init__(self, use_noise=False, suppress_info=False):
        """
        :param suppress_info: no log outputs to console
        """
        self._use_noise = use_noise
        self.suppress_info = suppress_info
        self._input_system = None
        self.robot = None
        self._DT = None

    def set_robot(self, robot):
        """Add a robot to simulate and specify output forms
        :param robot: robot to simulate (child of RobotSystem)
        :param outputs: tuple of outputs (child of OutputSystem)
        """
        self.robot = robot
        if robot._DT is not None:
            self._DT = robot._DT

    def run(self, display_output=None):
        """Run robots with the given settings for max_duration seconds
        :param max_duration: the maximum duration of test
        """
        t = 0
        """
        errors stored as tuple of (error, line_section)
        """
        errors = []
        while(True):
            inpt = (0.0, 0.0)
            try: 
                state = self.robot.drive(inpt, t)
                assert_space(state, STATE_SPACE)
                                    
                errors.append((self.robot.get_error(), self.robot.get_line_section()))

                outpt = self.robot.sense()
                assert_space(outpt, OUTPUT_SPACE)
            except Exception as e:
                # fail
                return None

            if(display_output):
                display_output.process(state, inpt, outpt, t)

            # check end of run
            if(self.fail()):
                return []
            elif(self.success()):
                return errors

            """
            if not self.suppress_info:
                logging.info("t: {}, inpt: {}, state: {}, outpt: {}".format(t, inpt, state, outpt))
            """

            t = self._step_forward(t, wait=display_output)

        #self.make_outputs()

    def success(self):
        return self.robot.finished()

    def fail(self):
        MAX_ERROR = 5
        MAX_READING = 5
        return self.robot.get_error() > MAX_ERROR or self.robot.last_reading_time > MAX_READING

    def _step_forward(self, t, wait=False):
        if self._DT is None:
            t += TEST.DT
            if(wait):
                time.sleep(TEST.DT)
        else:
            t = self._DT(t)
        return t

    def make_outputs(self):
        return None

