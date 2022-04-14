from sim.inputs.InputSystem import InputSystem
from pynput.keyboard import Key, Listener


class KEY:
    LEFT = 0
    RIGHT = 1
    UP = 2
    DOWN = 3


class KeyboardInput(InputSystem):
    def __init__(self):
        super().__init__()
        self._arrow = [False, False, False, False]

        self._start_capture_key()   # set listener

    def get_inputs(self, timestamp=None):
        self._key_handler() # update inpts based on current key states
        return self._inpts

    def is_done(self):
        return self._done

    def _start_capture_key(self):
        # listener for press/release events
        def on_press(key):
            if key == Key.esc:
                self._done = True
                return False    # kill listener
            elif key == Key.left:
                self._arrow[KEY.LEFT] = True
            elif key == Key.right:
                self._arrow[KEY.RIGHT] = True
            elif key == Key.up:
                self._arrow[KEY.UP] = True
            elif key == Key.down:
                self._arrow[KEY.DOWN] = True

        def on_release(key):
            if key == Key.esc:
                self._done = True
                return False    # kill listener
            elif key == Key.left:
                self._arrow[KEY.LEFT] = False
            elif key == Key.right:
                self._arrow[KEY.RIGHT] = False
            elif key == Key.up:
                self._arrow[KEY.UP] = False
            elif key == Key.down:
                self._arrow[KEY.DOWN] = False


        listener = Listener(on_press=on_press, on_release=on_release)
        listener.start()


    def _key_handler(self):
        """Defines how the robot moves"""
        if self._arrow[KEY.UP] and self._arrow[KEY.LEFT]:
            inpt = (0.5, 1.0)
        elif self._arrow[KEY.UP] and self._arrow[KEY.RIGHT]:
            inpt = (1.0, 0.5)
        elif self._arrow[KEY.DOWN] and self._arrow[KEY.LEFT]:
            inpt = (-0.5, -1.0)
        elif self._arrow[KEY.DOWN] and self._arrow[KEY.RIGHT]:
            inpt = (-1.0, -0.5)
        elif self._arrow[KEY.UP]:
            inpt = (1.0, 1.0)
        elif self._arrow[KEY.DOWN]:
            inpt = (-1.0, -1.0)
        elif self._arrow[KEY.LEFT]:
            inpt = (0.0, 1.0)
        elif self._arrow[KEY.RIGHT]:
            inpt = (1.0, 0.0)
        else:
            inpt = (0.0, 0.0)

        self._inpts = inpt

