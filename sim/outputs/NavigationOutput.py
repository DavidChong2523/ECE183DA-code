from sim.outputs.OutputSystem import OutputSystem
from sim.constants import DATA
from sim.constants import ROBOT
from sim.formulation import *
import pandas as pd
import matplotlib.pyplot as plt
import pygame

import cv2
import numpy as np



def rotation_mat(theta):
    c, s = np.cos(theta), np.sin(theta)
    rot = np.array([[c, -s], [s, c]])
    return rot

"""
dim: (length, width)
center: (x, y)
angle: counterclockwise from x axis
"""
def rotated_rect(dim, center, angle):
    l, w = dim
    center = np.array([[center[0]], [center[1]]])
    vertices = [(-w/2, l/2), (w/2, l/2), (w/2, -l/2), (-w/2, -l/2)]
    vertices = [np.array([[v[0]], [v[1]]]) for v in vertices]
    vertices = [rotation_mat(angle) @ v for v in vertices]
    vertices = [v + center for v in vertices]
    return vertices

def rotated_line(length, center, angle):
    center = np.array([[center[0]], [center[1]]])
    vertices = [(0, 0), (length, 0)]
    vertices = [np.array([[v[0]], [v[1]]]) for v in vertices]
    vertices = [rotation_mat(angle) @ v for v in vertices]
    vertices = [v + center for v in vertices]
    return vertices

def draw_rect(image, dim, center, angle, color=(255,0,0), thickness=1):
    vertices = rotated_rect(dim, center, angle)
    vertices = [(int(v[0][0]), int(v[1][0])) for v in vertices]

    for i in range(4):
        cv2.line(image, vertices[i], vertices[(i+1)%4], color, thickness)
    return image

def draw_line(image, length, center, angle, color=(0, 255, 0)):
    vertices = rotated_line(length, center, angle)
    vertices = [(int(v[0][0]), int(v[1][0])) for v in vertices]

    cv2.arrowedLine(image, vertices[0], vertices[1], color)
    return image
    
    
pygame.init()
vec = pygame.math.Vector2  # 2 for two dimensional

HEIGHT = 450
WIDTH = 400
ACC = 0.5
FRIC = -0.12
FPS = 60

FramePerSec = pygame.time.Clock()

displaysurface = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Game")

image = cv2.imread("line.png")
image = cv2.resize(image, (500, 500), interpolation=cv2.INTER_LINEAR)
image = ~image

class Player(pygame.sprite.Sprite):
    def __init__(self):
        super().__init__() 
        self.surf = pygame.Surface((30, 30))
        self.surf.fill((128,255,40))
        self.rect = self.surf.get_rect(center = (10, 420))

        self.pos = vec((10, 385))
        self.angle = 0
        self.vel = vec(0,0)
        self.acc = vec(0,0)

P1 = Player()

all_sprites = pygame.sprite.Group()
all_sprites.add(P1)

class NavigationOutput(OutputSystem):
    def __init__(self):
        super().__init__()
        self.image = np.zeros((500, 500, 3)).astype(np.uint8)
        self.fig = plt.figure()



        
    def process(self, state, inpt, outpt, timestamp):
        """save data to dynamic memory
        :param state: state space
        :param inpt: input space
        :param outpt: output space
        :param timestamp: corresponding timestamp
        """

        """save data to dynamic memory"""
        super().process(state, inpt, outpt, timestamp)
        
        X_IND = 0
        Y_IND = 1
        THETA_IND = 2
        THETA_DOT_IND = 3
        SX = 4
        SY = 5
        
        pos = (state[X_IND]+200, state[Y_IND]+200)
        sensor_pos = (state[SX]+200, state[SY]+200)
        angle = state[THETA_IND]
        #print("pos", pos)
        self.image = np.zeros((500, 500, 3))
        self.image = image.copy()
        self.image = draw_rect(self.image, (ROBOT.LENGTH, ROBOT.WIDTH), pos, angle)
        self.image = draw_rect(self.image, (ROBOT.SENSOR_LENGTH, ROBOT.SENSOR_WIDTH), sensor_pos, angle, color=(255, 255, 0), thickness=2)
        self.image = draw_line(self.image, 10, pos, angle)
        
        #plt.draw()
        #plt.pause(0.001)
        #cv2.imshow("test", self.image)
        #cv2.waitKey(1)
        #plt.imshow(self.image)
        #plt.show()

        surface = pygame.surfarray.make_surface(self.image)
        #P1.pos = vec(pos)
        #P1.rect.midbottom = P1.pos
        #P1.angle = angle
        displaysurface.fill((0,0,0))
        displaysurface.blit(surface, (0, 0))
        #for entity in all_sprites:
        #    displaysurface.blit(entity.surf, entity.rect)
    
        pygame.display.update()
        FramePerSec.tick(FPS)

    def make_output(self):
        """make proper output from the data"""
        #cv2.waitKey(0)
        #cv2.destroyAllWindows()
        pass
