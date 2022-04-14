

class InputSystem:
    def __init__(self):
        # private variables
        self._timestamps = []
        self._inpts = []
        self._done = False

    def get_inputs(self, timestamp=None):
        raise NotImplementedError

    def is_done(self):
        raise NotImplementedError
