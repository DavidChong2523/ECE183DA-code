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

cv2.imshow("test", image)
cv2.waitKey(0)
cv2.destroyAllWindows()