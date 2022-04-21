import cv2
import numpy as np
from sim.constants import *

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
    
def test():
    image = cv2.imread("line.png")
    image = cv2.resize(image, (500, 500), interpolation=cv2.INTER_LINEAR)
    image = ~image
    orig_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    pos = (200, 200)
    pos = real2image(pos, image.shape, xy=True)
    sensor_pos = (ROBOT.SENSOR_POS[0] + 200, ROBOT.SENSOR_POS[1] + 200)
    sensor_pos = real2image(sensor_pos, image.shape, xy=True)
    angle = 0
    angle = angle2image(angle)

    # simulate sensing
    # rotate image opposite sensor angle
    orig_image[int(sensor_pos[1])][int(sensor_pos[0])] = 1
    sensor_dim_x = 1 # 1/2 inch by 1/2 inch
    sensor_dim_y = 1 
    scol, srow = int(sensor_pos[0]), int(sensor_pos[1])
    r1, r2 = srow-sensor_dim_y, srow+sensor_dim_y
    c1, c2 = scol-sensor_dim_x, scol+sensor_dim_x
    sensed_array = orig_image[r1:r2, c1:c2]
    sensed_value = np.max(sensed_array)
    print(sensed_array)
    #print(sensed_array.shape, scol, srow, sensor_dim_x, sensor_dim_y, r1, r2, c1, c2)
    print(sensed_value)

    image = draw_rect(image, (ROBOT.LENGTH, ROBOT.WIDTH), pos, angle)
    image = draw_rect(image, (ROBOT.SENSOR_LENGTH, ROBOT.SENSOR_WIDTH), sensor_pos, angle, color=(255, 255, 0), thickness=2)
    image = draw_line(image, 10, pos, angle)

    cv2.imshow("tets", image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    test()