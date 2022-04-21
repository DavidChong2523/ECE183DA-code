from sim.outputs.OutputSystem import OutputSystem
from sim.constants import DATA
from sim.constants import ROBOT
from sim.formulation import *
import pandas as pd
import matplotlib.pyplot as plt
import pygame

import cv2
import numpy as np

#from sim.outputs.display import *

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
    # initial orientation parallel to x axis
    vertices = [(l/2, w/2), (l/2, -w/2), (-l/2, -w/2), (-l/2, w/2)]
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
    
### returns (x, y) format
def image2real(pix, image_shape):
    row, col = pix
    heigth, width = image_shape[0], image_shape[1]
    real_x = col
    real_y = height-row
    return (real_x, real_y)

### returns (row, col) format
def real2image(pos, image_shape, xy=False):
    x, y = pos
    col = x
    row = image_shape[0]-y
    if(xy):
        return (col, row)
    else:
        return (row, col)

def angle2image(angle):
    return (2*np.pi - angle) % (2*np.pi)
    

def sense(pos, angle, orig_image):
    sensor_pos = real2image(pos, orig_image.shape, xy=False)
    #angle = angle2image(angle)

    # rotate image to match angle 
    # TEST THIS!!!!
    rotate_matrix = cv2.getRotationMatrix2D(center=sensor_pos, angle=angle, scale=1)
    orig_image = cv2.warpAffine(src=orig_image, M=rotate_matrix, dsize=(orig_image.shape[1], orig_image.shape[0]))

    print(orig_image.shape)
    #orig_image = rotation_mat(angle) @ orig_image
    sensor_dim_x = 1 # 1/2 inch by 1/2 inch
    sensor_dim_y = 1 
    srow, scol = int(sensor_pos[0]), int(sensor_pos[1])
    r1, r2 = srow-sensor_dim_y, srow+sensor_dim_y
    c1, c2 = scol-sensor_dim_x, scol+sensor_dim_x
    sensed_array = orig_image[r1:r2, c1:c2]
    sensed_value = np.max(sensed_array)
    print("sensor_pos", sensor_pos)
    #print(sensed_array, sensed_value)
    #print(sensed_array.shape, scol, srow, sensor_dim_x, sensor_dim_y, r1, r2, c1, c2)
    return sensed_value

pygame.init()

HEIGHT = 450
WIDTH = 400
FPS = 60

FramePerSec = pygame.time.Clock()

displaysurface = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Game")

image = cv2.imread("line.png")
image = cv2.resize(image, (500, 500), interpolation=cv2.INTER_LINEAR)
image = ~image
image = np.vstack((image, np.zeros((100, 500, 3), np.uint8)))
print(image.shape)
orig_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
#cv2.imshow("test", image)
#cv2.waitKey(0)

class NavigationOutput(OutputSystem):
    def __init__(self):
        super().__init__()
        self.image = np.zeros((500, 500, 3)).astype(np.uint8)

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
        
        self.image = image.copy()

        pos = (state[X_IND]+300, state[Y_IND]+300)
        pos = real2image(pos, self.image.shape, xy=True)
        print("IMAGE pos", pos)
        
        angle = state[THETA_IND]

        base_sensor_pos = np.array([state[SX]+300, state[SY]+300])
        sense_vec = np.array([np.cos(angle-(np.pi/2)), np.sin(angle-(np.pi/2))])
        sensor_array = [base_sensor_pos+i*sense_vec for i in [-4, -2, 0, 2, 4]]

        # array sensing
        for i, sp in enumerate(sensor_array[::-1]):
            sp = (sp[0], sp[1])
            val = sense(sp, angle, orig_image)
            if(val == 0):
                color = (255, 100, 0)
            else:
                color = (0, 150, 255)
            x = 40*i + 50
            #cv2.rectangle(self.image, (x-25, 575), (x+25, 525), color, -1)
            cv2.rectangle(self.image, (400+20, x-12), (400+44, x+12), color, -1)

        angle = angle2image(angle)

        # simulate sensing
        # rotate image opposite sensor angle



        #self.image = np.zeros((600, 500, 3))
        self.image = draw_rect(self.image, (ROBOT.LENGTH, ROBOT.WIDTH), pos, angle)
        for sp in sensor_array:
            sp = (sp[0], sp[1])
            sensor_pos = real2image(sp, self.image.shape, xy=True)
            self.image = draw_rect(self.image, (ROBOT.SENSOR_LENGTH, ROBOT.SENSOR_WIDTH), sensor_pos, angle, color=(255, 255, 0), thickness=2)
        self.image = draw_line(self.image, 10, pos, angle)


        # reflect image to display properly in pygame
        #spygame_image  np.zeros(self.image.shape)
        #pygame_image = np.transpose(self.image[::-1],axes=[1,0,2])#[::-1]
        #pygame_image[2] = self.
        #print(pygame_image.shape, self.image.shape)
        pygame_image = self.image
        surface = pygame.surfarray.make_surface(pygame_image)
        
        #P1.pos = vec(pos)
        #P1.rect.midbottom = P1.pos
        #P1.angle = angle
        displaysurface.fill((0,0,0))
        displaysurface.blit(surface, (0, 0))
        #for entity in all_sprites:
        #    displaysurface.blit(entity.surf, entity.rect)
    
        pygame.display.update()
        FramePerSec.tick(FPS)

        # do sensing

    def make_output(self):
        """make proper output from the data"""
        #cv2.waitKey(0)
        #cv2.destroyAllWindows()
        pass
