import logging, os

from sim.Simulation import Sim
from sim.TestSim import TestSim
from sim.inputs import FileInput, KeyboardInput, JoystickInput
from sim.outputs import FileOutput, GraphOutput, AnimationOutput, NavigationOutput
from sim.robots import NOPModel, KinematicsModel
from sim.environment import LineEnv
from matplotlib.colors import ListedColormap
import seaborn as sns

import cv2
import pandas as pd
import numpy as np
import itertools
import matplotlib.pyplot as plt
import time


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
    """
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
    """

    noise_params = {
        "dirt_prob": 0.0,
        "grass_prob": 0.0
    }
    robot_noise = {
        "trans_noise": 0.0,
        "rot_noise": 0,
        "trans_mag": 1,
        "rot_mag": 0
    }
    env = LineEnv.LineEnv("line_3ft_turn_labeled.png", noise_params=noise_params)
    kinematics_model = KinematicsModel(env, noise_params=robot_noise)
    out = NavigationOutput.NavigationOutput(kinematics_model, env)
    s.add_robot(kinematics_model, (out,))

    # Finally to run the simulation
    s.run(max_duration=None)  # run 10sec, at the end of run, automatically do outputs.

def autoSim():
    start = time.time()
    REPEATS = 10
    s = TestSim(suppress_info=True)

    out_df = pd.DataFrame()

    """
    final testing:
    dirt_vals = np.linspace(0, 1, num=10)
    grass_vals = np.linspace(0, 0.3, num=10)
    actuation_vals = np.linspace(0, 2.5, num=10)
    """
    dirt_vals = np.linspace(0, 1, num=1)
    grass_vals = np.linspace(0.15, 0.3, num=1)
    actuation_vals = np.linspace(0, 2.5, num=1)
    for d, g, a in itertools.product(dirt_vals, grass_vals, actuation_vals): 
        """
        env_noise_params = {
            "dirt_prob": 0.01,
            "dirt_a1": d,
            "dirt_a2": d+2,
            "dirt_area": 0,
            "grass_prob": 0.001,
            "grass_a1": g,
            "grass_a2": g+2,
            "grass_area": 0
        }
        """
        env_noise_params = {
            "dirt_prob": d,
            "grass_prob": g,
        }

        robot_noise_params = {
            "trans_noise": 0.1,
            "rot_noise": 0.0,
            "trans_mag": a,
            "rot_mag": 0
        }

        straight_errors = []
        curve_errors = []
        num_fails = 0
        for k in range(REPEATS):
            env = LineEnv.LineEnv("line_4ft_turn_labeled.png", env_noise_params)
            robot = KinematicsModel(env, noise_params=robot_noise_params)
            #out = NavigationOutput.NavigationOutput(robot, env)
            s.set_robot(robot)

            errors = s.run(display_output=None)
            if(not errors):
                num_fails += 1
            else:
                straight = [e[0] for e in errors if e[1]]
                curve = [e[0] for e in errors if not e[1]]
                straight_errors.append(np.average(straight))
                curve_errors.append(np.average(curve))
        if(len(straight_errors) == 0):
            straight_errors = [-1]
        if(len(curve_errors) == 0):
            curve_errors = [-1]
        percentage_success = (REPEATS-num_fails) / REPEATS
            
        print("Noise area:", "%.2f" % d, "%.2f" % g, "%.2f" % a, "Average Straight Error:", "%.2f" % np.average(straight_errors), "Average Curve Error:", "%.2f" % np.average(curve_errors), "Percentage Success:", "%.2f" % percentage_success)
        out_df = out_df.append(pd.DataFrame({"dirt_mag": [d], "grass_mag": [g], "actuation_mag": [a], "straight_error": [np.average(straight_errors)], "curve_error": [np.average(curve_errors)], "percentage_success": [percentage_success]}), ignore_index=True)
    #out_df.to_csv("test_2.csv")
    end = time.time()
    print("total time:", end-start)
    print(out_df)

def plot():
    out_df = pd.read_csv("test.csv")
    xvals, yvals = [], []
    for i, row in out_df.iterrows():
        if(row["error"] != -1):
            xvals.append(row["dirt_mag"])
            yvals.append(row["grass_mag"])
    
    xvals = np.array(xvals) / 8
    yvals = np.array(yvals) / 8
    plt.title("Successful Runs")
    plt.xlabel("Area of Dirt (False Negatives)")
    plt.ylabel("Area of Grass (False Positives)")
    plt.scatter(xvals, yvals)
    plt.show()

def plot3d(out_df):
    #out_df = pd.read_csv("test.csv")
    xvals, yvals, zvals = [], [], []
    cvals = []
    straight_error_vals = set()
    curve_error_vals = set()
    for i, row in out_df.iterrows():
        if(row["percentage_success"] > 0):
            xvals.append(row["dirt_mag"])
            yvals.append(row["grass_mag"])
            zvals.append(row["actuation_mag"])
            cvals.append(row["percentage_success"])
            straight_error_vals.add(row["straight_error"])
            curve_error_vals.add(row["curve_error"])
    print(max(straight_error_vals), min(straight_error_vals))
    print(max(curve_error_vals), min(curve_error_vals))
    fig = plt.figure()
    ax = plt.axes(projection="3d")
    ax.set_xlabel("Dirt Area (Percent)")
    ax.set_ylabel("Grass Area (Percent)")
    ax.set_zlabel("Actuation Noise (in.)")
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 0.3)
    ax.set_zlim(0, 2.5)
    ax.set_title("Percentage of Trials Successful")
    cmap = ListedColormap(['tab:brown', 'tab:gray', 'tab:purple', 'tab:blue', 'tab:cyan', 'tab:green', 'tab:olive', 'tab:orange', 'tab:pink', 'tab:red'])#'sns.color_palette("husl", 256).as_hex())
    
    # plotting a scatter plot with X-coordinate,
    # Y-coordinate and Z-coordinate respectively
    # and defining the points color as cividis
    # and defining c as z which basically is a
    # defination of 2D array in which rows are RGB
    #or RGBA
    sc = ax.scatter3D(xvals, yvals, zvals, c=cvals, cmap=cmap)#'cividis')

    plt.legend(*sc.legend_elements(), bbox_to_anchor=(1.05, 1), loc=2)
    
    # Showing the above plot
    #plt.show()

def plot_errors():
    out_df = pd.read_csv("test_2.csv")
    xvals, yvals, zvals = [], [], []
    cvals = []
    for i, row in out_df.iterrows():
        if(row["percentage_success"] > 0):
            xvals.append(row["dirt_mag"])
            yvals.append(row["grass_mag"])
            zvals.append(row["actuation_mag"])
            cvals.append(row["straight_error"])
            #cvals.append(row["curve_error"])
    fig = plt.figure()
    ax = plt.axes(projection="3d")
    ax.set_xlabel("Dirt Area (Percent)")
    ax.set_ylabel("Grass Area (Percent)")
    ax.set_zlabel("Actuation Noise (in.)")
    ax.set_xlim(0, 0.6)
    ax.set_ylim(0, 0.3)
    ax.set_zlim(0, 2.5)
    ax.set_title("Successful Runs")
    colors = ['tab:brown', 'tab:gray', 'tab:purple', 'tab:blue', 'tab:cyan', 'tab:green', 'tab:olive', 'tab:orange', 'tab:pink', 'tab:red']
    cmap = ListedColormap(colors[::-1])#'sns.color_palette("husl", 256).as_hex())
    
    # plotting a scatter plot with X-coordinate,
    # Y-coordinate and Z-coordinate respectively
    # and defining the points color as cividis
    # and defining c as z which basically is a
    # defination of 2D array in which rows are RGB
    #or RGBA
    sc = ax.scatter3D(xvals, yvals, zvals, c=cvals, cmap=cmap)#'cividis')

    plt.legend(*sc.legend_elements(), bbox_to_anchor=(1.05, 1), loc=2)
    
    # Showing the above plot
    plt.show()


def plot_errors(out_df):
    xvals, yvals, zvals = [], [], []
    cvals = []
    for i, row in out_df.iterrows():
        if(row["percentage_success"] > 0):
            xvals.append(row["dirt_mag"])
            yvals.append(row["grass_mag"])
            zvals.append(row["actuation_mag"])
            cvals.append(row["straight_error"])
            #cvals.append(row["curve_error"])
    fig = plt.figure()
    ax = plt.axes(projection="3d")
    ax.set_xlabel("Dirt Area (Percent)")
    ax.set_ylabel("Grass Area (Percent)")
    ax.set_zlabel("Actuation Noise (in.)")
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 0.3)
    ax.set_zlim(0, 2.5)
    ax.set_title("Average Error During Straight Sections")
    colors = ['tab:brown', 'tab:gray', 'tab:purple', 'tab:blue', 'tab:cyan', 'tab:green', 'tab:olive', 'tab:orange', 'tab:pink', 'tab:red']
    cmap = ListedColormap(colors[::-1])#'sns.color_palette("husl", 256).as_hex())
    
    # plotting a scatter plot with X-coordinate,
    # Y-coordinate and Z-coordinate respectively
    # and defining the points color as cividis
    # and defining c as z which basically is a
    # defination of 2D array in which rows are RGB
    #or RGBA
    sc = ax.scatter3D(xvals, yvals, zvals, c=cvals, cmap=cmap)#'cividis')

    plt.legend(*sc.legend_elements(), bbox_to_anchor=(1.05, 1), loc=2)
    
    # Showing the above plot
    #plt.show()


if __name__ == "__main__":
    while(True):
        key = input(">>>")
        if(key == "s"):
            standardSim()
        elif(key == "a"):
            autoSim()
            #print("auto sim not active")
        #elif(key == "x"):
        #    plot()
        elif(key == "x"):
            plot3d(pd.read_csv("test.csv"))
            plot3d(pd.read_csv("test_2.csv"))
            plt.show()
        elif(key == "y"):
            plot_errors(pd.read_csv("test.csv"))
            plot_errors(pd.read_csv("test_2.csv"))
            plt.show()
        #plot3d()
        #plot_errors()