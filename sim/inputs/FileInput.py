from sim.inputs.InputSystem import InputSystem
from sim.constants import DATA
from sim.formulation import INPUT_SPACE
import pandas as pd


class FileInput(InputSystem):
    def __init__(self, filepath):
        super().__init__()
        self._filepath = filepath
        # required call
        self._open_file()

    def get_inputs(self, timestamp=None):
        return self._find_input_by_timestamp(timestamp)

    def is_done(self):
        return self._done


    def _open_file(self):
        df = pd.read_csv(self._filepath)
        df_dict = df.to_dict('list')
        self._timestamps = df_dict[DATA.TIMESTAMP]
        self._inpts = tuple(zip(*(df_dict[k] for k in INPUT_SPACE.keys()))) #zipping 2 lists to create list of tuple

    def _find_input_by_timestamp(self, timestamp):
        # Zero-order hold: return most recently specified inputs
        inpt = None
        for t, i in zip(self._timestamps, self._inpts):
            if t > timestamp:
                return inpt
            inpt = i

        self._done = True
        return inpt
