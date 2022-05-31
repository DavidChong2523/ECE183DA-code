from sim.inputs.InputSystem import InputSystem

class NullInput(InputSystem):
    def __init__(self):
        super().__init__()

    def get_inputs(self, timestamp=None):
        return (0.0, 0.0)

    def is_done(self):
        return False