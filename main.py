import logging, os

from sim.Simulation import Sim
from sim.TestSim import TestSim
from sim.inputs import FileInput, KeyboardInput, JoystickInput
from sim.outputs import FileOutput, GraphOutput, AnimationOutput, NavigationOutput
from sim.robots import NOPModel, KinematicsModel
from sim.environment import LineEnv

import cv2
import numpy as np

def standardSim():
    LOGLEVEL = os.environ.get('LOGLEVEL', 'INFO').upper()
    logging.basicConfig(level=LOGLEVEL)

    s = Sim(suppress_info=True)    # Create instance of Robot testing system

    # Create instance of inputs system.
    # You can only have one type of inputs per test
    i = KeyboardInput()
    s.set_input(i)  

    # Create instance of robots and corresponding output methods.
    # each robot can have multiple output system
    noise_params = {
        "dirt_prob": 0.00,
        "dirt_a1": 20,
        "dirt_a2": 50,
        "dirt_area": 0,
        "grass_prob": 0.000,
        "grass_a1": 30,
        "grass_a2": 50,
        "grass_area": 0
    }
    env = LineEnv.LineEnv("line_1ft_turn_labeled.png", noise_params=noise_params)
    kinematics_model = KinematicsModel(env)
    out = NavigationOutput.NavigationOutput(kinematics_model, env)
    s.add_robot(kinematics_model, (out,))

    # Finally to run the simulation
    s.run(max_duration=None)  # run 10sec, at the end of run, automatically do outputs.

def autoSim():
    REPEATS = 4
    s = TestSim(suppress_info=True)

    for i in range(1, 11):
        noise_params = {
            "dirt_prob": 0.01,
            "dirt_a1": i*10,
            "dirt_a2": i*10+1,
            "dirt_area": 0,
            "grass_prob": 0.000,
            "grass_a1": 20,
            "grass_a2": 40,
            "grass_area": 0
        }

        errors = []
        for j in range(REPEATS):
            env = LineEnv.LineEnv("line_4ft_turn_labeled.png", noise_params)
            robot = KinematicsModel(env)
            #out = NavigationOutput.NavigationOutput(robot, env)
            s.set_robot(robot)

            avg_error = s.run(display_output=None)
            if(not avg_error):
                errors = [-1]
                break
            else:
                errors.append(avg_error)
        print("Noise area:", i*10, "Average Error:", np.average(errors))
if __name__ == "__main__":
    #standardSim()
    autoSim()