import cv2
import numpy as np
from sim.constants import *

"""
pos: numpy array [x, y] in real coords
returns: numpy array [x, y] in image coords
"""
def real2image(pos):
    pos = pos * np.array([1, -1])
    return pos

"""
pos: numpy array [x, y] in image coords
returns: numpy array [x, y] in real coords
"""
def image2real(pos):
    pos = pos * np.array([1, -1])
    return pos

"""
theta: angle in radians
"""
def rotation_mat(theta):
    c, s = np.cos(theta), np.sin(theta)
    rot = np.array([[c, -s], [s, c]])
    return rot

"""
dim: (length, width) in image scale
center: np array [x, y] in image coords
angle: radians
returns: vector of corner positions in image coords
"""
def rotated_rect(dim, center, angle):
    l, w = dim
    # initial orientation parallel to x axis
    vertices = [[l/2, w/2], [l/2, -w/2], [-l/2, -w/2], [-l/2, w/2]]
    vertices = [np.array(v) for v in vertices]
    vertices = [rotation_mat(angle) @ v for v in vertices]
    vertices = [real2image(v) + center for v in vertices]
    return vertices

"""
length: distance in image scale
center: numpy array [x, y] in image coords
angle: radians
returns: vector of start and end positions in image coords
"""
def rotated_line(length, center, angle):
    vertices = [np.array([0, 0]), np.array([length, 0])]
    vertices = [rotation_mat(angle) @ v for v in vertices]
    vertices = [real2image(v) + center for v in vertices]
    return vertices

"""
dim, center, angle: same params as rotated_rect()
"""
def draw_rect(image, dim, center, angle, color=(255,0,0), thickness=1):
    vertices = rotated_rect(dim, center, angle)
    vertices = [(int(v[0]), int(v[1])) for v in vertices]
    for i in range(4):
        cv2.line(image, vertices[i], vertices[(i+1)%4], color, thickness)
    return image

"""
length, center, angle: same params as rotated_line()
"""
def draw_line(image, length, center, angle, color=(0, 255, 0), thickness=1):
    vertices = rotated_line(length, center, angle)
    vertices = [(int(v[0]), int(v[1])) for v in vertices]

    cv2.arrowedLine(image, vertices[0], vertices[1], color, thickness=thickness)
    return image

"""
center: numpy array [x, y] image coords
angle: radians
"""
def rotate_image(image, center, angle):
    center = np.rint(center)
    center = (int(center[0]), int(center[1]))
    angle = np.rad2deg(angle)
    dims = (image.shape[1], image.shape[0])

    rotate_matrix = cv2.getRotationMatrix2D(center=center, angle=angle, scale=1)
    image = cv2.warpAffine(src=image, M=rotate_matrix, dsize=dims)
    return image

"""
// NOT CURRENTLY env: grayscale image [0, 255]
pos: numpy array [x, y] in image coords
angle: radians
dim: (length, width) in image scale
noise: float [0, 1], probability of flipping the sensor reading
returns: bool, True if line sensed, False otherwise
"""
def sensor_reading(env, pos, angle, dim, noise=0):
    # rotate environment
    rot = (np.pi/2) - angle
    rot_env = rotate_image(env, pos, rot)

    l, w = dim
    c1, c2 = int(np.rint(pos[0]-w/2)), int(np.rint(pos[0]+w/2))
    r1, r2 = int(np.rint(pos[1]-l/2)), int(np.rint(pos[1]+l/2))
    sensed_area = rot_env[r1:r2+1, c1:c2+1]
    sensed_area = cv2.cvtColor(sensed_area, cv2.COLOR_BGR2GRAY)

    area = sensed_area.shape[0]*sensed_area.shape[1]    
    reading = np.count_nonzero(sensed_area) > area / 2

    # add noise to reading
    if(np.random.uniform() < noise):
        reading = not reading

    return reading

"""
real_dist: float representing dist in inches
"""
def image_scale(real_dist):
    # scale: 1 px = 0.01 inches
    scale = 1 / 0.01
    return scale * real_dist


def scale_image(image):
    scaling = int(ENVIRONMENT.IMAGE_SCALE / ENVIRONMENT.INPUT_SCALE)
    new_dim = (scaling*image.shape[0], scaling*image.shape[1])
    print(new_dim)
    scaled_image = cv2.resize(image, new_dim, cv2.INTER_NEAREST)
    return scaled_image

"""
pos: numpy array [x, y] real coords
start_image: numpy array [x, y] image coords
"""
def env2image(pos, start_image):
    pos = real2image(pos)
    pos = pos * ENVIRONMENT.DISPLAY_SCALE
    pos = pos + start_image
    return pos