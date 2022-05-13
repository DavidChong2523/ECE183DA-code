import logging, os

from sim.Simulation import Sim
from sim.inputs import FileInput, KeyboardInput, JoystickInput
from sim.outputs import FileOutput, GraphOutput, AnimationOutput, NavigationOutput
from sim.robots import NOPModel, KinematicsModel
from sim.environment import LineEnv

import cv2
LOGLEVEL = os.environ.get('LOGLEVEL', 'INFO').upper()
logging.basicConfig(level=LOGLEVEL)

s = Sim(suppress_info=True)    # Create instance of Robot testing system

# Create instance of inputs system.
# You can only have one type of inputs per test
#i = FileInput('test.csv')
i = KeyboardInput()
#i = JoystickInput()

s.set_input(i)  # specify inputs to run

# Create instance of robots and corresponding output methods.
# each robot can have multiple output system
image = LineEnv.LineEnv("line_4ft_turn.png")
image = image.image
kinematics_model = KinematicsModel(image)
#kin_out1 = FileOutput('test_kinematics.csv')
#kin_out2 = GraphOutput(figname="test_kinematics.png")

kin_out3 = NavigationOutput.NavigationOutput(kinematics_model, image)

# if you like to output in several different ways
# nop_out2 = GraphOutput()
# s.add_robot(nop_model, (nop_out1, nop_out2))

s.add_robot(kinematics_model, (kin_out3,))

# Finally to run the simulation
s.run(max_duration=None)  # run 10sec, at the end of run, automatically do outputs.
