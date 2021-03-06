from sim.constants import DATA
from sim.formulation import *


class OutputSystem:
    def __init__(self):
        self._data = []
        self._states = []
        self._outpts = []
        self._inpts = []
        self._timestamps = []

    def process(self, state, inpt, outpt, timestamp):
        """save data to dynamic memory
        :param state: state space
        :param inpt: input space
        :param outpt: output space
        :param timestamp: corresponding timestamp
        """
        self._timestamps.append(timestamp)
        a = self._handle_new_data(inpt, self._inpts, INPUT_SPACE)
        s = self._handle_new_data(state, self._states, STATE_SPACE)
        o = self._handle_new_data(outpt, self._outpts, OUTPUT_SPACE)

        d = {DATA.TIMESTAMP: timestamp, **a , **s, **o}
        self._data.append(d)


    def _handle_new_data(self, newdata, alldata, space):
        assert_space(newdata, space)
        d = dict(zip(space, newdata))
        alldata.append(d)
        return d


    def make_output(self):
        """make proper output from the data"""
        raise NotImplementedError
