from sim.outputs.OutputSystem import OutputSystem
from sim.constants import DATA
from sim.constants import ROBOT
from sim.constants import *
from sim.formulation import *
import pandas as pd
import matplotlib.pyplot as plt
import pygame
import sim.utils as utils
import cv2
import numpy as np

HEIGHT = 600
WIDTH = 600
FPS = 60



class NavigationOutput(OutputSystem):
    def __init__(self, robot, environment):
        super().__init__()
        self.environment = environment
        self.image = environment.image
        self.robot = robot

        pygame.init()
        self.FramePerSec = pygame.time.Clock()
        self.displaysurface = pygame.display.set_mode((WIDTH, HEIGHT))
        pygame.display.set_caption("Navigation Simulation")

        self.fig, self.axs = plt.subplots(2, 1)
        self.error = []
        self.avg_error = []

    def info_bar(self, sensor_readings):
        IB_HEIGHT = 100
        GRAY = (25, 25, 25)
        info_bar = np.zeros((IB_HEIGHT, self.image.shape[1], 3), np.uint8)
        info_bar[:,:,:] = GRAY

        # draw sensor readings
        block_dim = 20
        block_spacing = 10
        block_y = 50
        start_x = 50
        for i, sr in enumerate(sensor_readings):
            if(sr == 1):
                color = (255, 100, 0)
            else:
                color = (0, 150, 255)
            block_x = start_x + i*(block_dim+block_spacing)
            top_left = (block_x-block_dim//2, block_y-block_dim//2)
            bot_right = (block_x+block_dim//2, block_y+block_dim//2)
            cv2.rectangle(info_bar, top_left, bot_right, color, -1)

        return info_bar

    def plot_pid(self, pid_error_history):
        self.axs[0].set_title("P value")
        self.axs[0].plot(pid_error_history, color="blue")

        self.error.append(self.robot.get_error())
        self.avg_error.append(np.average(self.error))
        self.axs[1].set_title("Error")
        self.axs[1].plot(self.error, color="blue", label="error")
        self.axs[1].plot(self.avg_error, color="orange", label="avg error")
        #self.axs[1].legend()

        plt.pause(0.05)
        #plt.show()

    def display(self, state, inpt, outpt, timestamp):
        pygame.event.pump()
        x = state[ROBOT.I_X]
        y = state[ROBOT.I_Y]
        theta = state[ROBOT.I_THETA]
        sensor_array = self.robot.get_sensor_positions()
        
        display_image = self.image.copy()

        # upscale region around robot
        DIR_LEN = 30 
        image_pos = utils.env2image(np.array([x, y]), self.robot.image_start)
        image_len = ENVIRONMENT.DISPLAY_SCALE * ROBOT.LENGTH
        image_width = ENVIRONMENT.DISPLAY_SCALE * ROBOT.WIDTH

        # draw robot
        display_image = utils.draw_rect(display_image, (image_len, image_width), image_pos, theta, thickness=3)
        display_image = utils.draw_line(display_image, DIR_LEN, image_pos, theta, thickness=3)

        # draw sensors
        SENSOR_COLOR = (255, 255, 0)
        SENSOR_THICKNESS = 2
        for spos in sensor_array:
            image_pos = utils.env2image(spos, self.robot.image_start)
            image_len = ENVIRONMENT.DISPLAY_SCALE * ROBOT.SENSOR_LENGTH
            image_width = ENVIRONMENT.DISPLAY_SCALE * ROBOT.SENSOR_WIDTH
            display_image = utils.draw_rect(display_image, (image_len, image_width), image_pos, theta, color=SENSOR_COLOR, thickness=SENSOR_THICKNESS)
        
        # draw info bar
        info_bar = self.info_bar(outpt[ROBOT.I_SENSE])
        display_image = np.vstack((display_image, info_bar))
                
        # display image
        #display_image = cv2.cvtColor(display_image, cv2.COLOR_BGR2RGB)
        display_image = cv2.rotate(display_image, cv2.ROTATE_90_CLOCKWISE)
        display_image = cv2.flip(display_image, 1)
        display_image = cv2.resize(display_image, (HEIGHT, WIDTH))
        surface = pygame.surfarray.make_surface(display_image)
        self.displaysurface.fill((0,0,0))
        self.displaysurface.blit(surface, (0, 0))
        pygame.display.flip()
        self.FramePerSec.tick(FPS)

    def process(self, state, inpt, outpt, timestamp):
        """save data to dynamic memory
        :param state: state space
        :param inpt: input space
        :param outpt: output space
        :param timestamp: corresponding timestamp
        """

        """save data to dynamic memory"""
        super().process(state, inpt, outpt, timestamp)
        self.display(state, inpt, outpt, timestamp)
        #self.plot_pid(self.robot.pid_error_history)

    def make_output(self):
        """make proper output from the data"""
        pass
