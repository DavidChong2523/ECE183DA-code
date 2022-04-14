from sim.inputs.InputSystem import InputSystem
import logging
import pygame


class JOY:
    STICK_LEFT_X = 0
    STICK_LEFT_Y = 1
    STICK_RIGHT_X = 2
    STICK_RIGHT_Y = 3
    THROTTLE_L = 4
    THROTTLE_R = 5

    BUTTON_A = 0
    BUTTON_B = 1
    BUTTON_X = 2
    BUTTON_Y = 3
    BUTTON_LB = 4
    BUTTON_RB = 5

    SENSITIVITY_HIGH = 3
    SENSITIVITY_MEDIUM = 2
    SENSITIVITY_LOW = 1

    DEADZONE = [0.15, 0.15, 0.25, 0.15, 0.0, 0.0]


class JoystickInput(InputSystem):
    def __init__(self):
        super().__init__()
        self._Id = 0
        # for pygame
        pygame.init()
        pygame.joystick.init()
        try:
            self._joystick = pygame.joystick.Joystick(0)
            self._joystick.init()
            self.stick_name = self._joystick.get_name()
            logging.info("Joystick name: {}".format(self.stick_name))
        except:
            logging.error("Connect Controller!")
            self.close = True

    def __del__(self):
        pygame.quit()

    def get_inputs(self, timestamp=None):
        self._capture_joystick()
        return self._inpts

    def is_done(self):
        return self._done


# internal

    def _capture_joystick(self):
        pygame.event.pump()
        joystick = self._joystick

        # capture joystick values
        num_axes = joystick.get_numaxes()
        self.axes = []
        for i in range(num_axes):
            self.axes.append(joystick.get_axis(i))
        # capture button status
        num_button = joystick.get_numbuttons()
        self.buttons = []
        for i in range(num_button):
            self.buttons.append(joystick.get_button(i))

        self._stick_handler()
        self._button_handler()

    def _stick_handler(self):
        self._filter_stick()    # create deadzone because stick don't go back to 0
        self._inpts = (self.axes[JOY.STICK_LEFT_Y], self.axes[JOY.STICK_RIGHT_Y])

    def _button_handler(self):
        if self.buttons[JOY.BUTTON_X]:
            self._done = True
            logging.info("X button pressed to close")

    def _filter_stick(self):
        """analog stick needs to be filtered out for deadzone"""
        dead = JOY.DEADZONE
        for i in range(len(self.axes)):
            if -dead[i] < self.axes[i] < dead[i]:
                self.axes[i] = 0.0
