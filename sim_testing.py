import cv2
import numpy as np
from sim.constants import *
import sim.utils as utils

"""
pos: numpy array [x, y] in real coords
returns: numpy array [x, y] in image coords
"""
def real2image(pos):
    pos[1] *= -1
    return pos

"""
pos: numpy array [x, y] in image coords
returns: numpy array [x, y] in real coords
"""
def image2real(pos):
    pos[1] *= -1
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
def draw_line(image, length, center, angle, color=(0, 255, 0)):
    vertices = rotated_line(length, center, angle)
    vertices = [(int(v[0]), int(v[1])) for v in vertices]

    cv2.arrowedLine(image, vertices[0], vertices[1], color)
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
pos: numpy array [x, y] in image coords
angle: radians
dim: (length, width) in image scale
"""
def sensor_reading(env, pos, angle, dim):
    # rotate environment
    rot = (np.pi/2) - angle
    rot_env = rotate_image(env, pos, rot)

    l, w = dim
    c1, c2 = int(np.rint(pos[0]-w/2)), int(np.rint(pos[0]+w/2))
    r1, r2 = int(np.rint(pos[1]-l/2)), int(np.rint(pos[1]+l/2))
    sensed_area = rot_env[r1:r2+1, c1:c2+1, :]
    sensed_area = cv2.cvtColor(sensed_area, cv2.COLOR_BGR2GRAY)
    area = sensed_area.shape[0]*sensed_area.shape[1]
    reading = np.count_nonzero(sensed_area) > area / 2
    return reading

"""
real_dist: float representing dist in inches
"""
def image_scale(real_dist):
    # scale: 1 px = 0.01 inches
    scale = 1 / 0.01
    return scale * real_dist
    
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

def test2():
    img = np.zeros((500, 600, 3), np.uint8)
    img[:, 500:, 1] = 255
    cv2.rectangle(img, (200, 200), (250, 300), color=(255, 0, 0), thickness=-1)
    cv2.rectangle(img, (220, 220), (230, 230), color=(255, 255, 0), thickness=-1)
    cv2.line(img, (100, 300), (100, 100), color=(0,0,255), thickness=1)
    img = draw_line(img, 20, np.array([225, 225]), np.pi*3/2, color=(0, 255, 255))
    
    ang = np.pi/6
    img = draw_rect(img, (10, 30), np.array([400, 400]), ang)
    img = draw_line(img, 10, np.array([400, 400]), ang)
    cv2.rectangle(img, (398, 398), (402, 402), color=(0, 255, 255), thickness = -1)
    angle = (np.pi/2 - ang)*180/np.pi#45#np.pi/2
    rotate_matrix = cv2.getRotationMatrix2D(center=(225, 225), angle=angle, scale=1)
    orig_image = cv2.warpAffine(src=img, M=rotate_matrix, dsize=(img.shape[1], img.shape[0]))
    print(sensor_reading(img, np.array([400.5, 400.5]), ang, (1,1)))
    cv2.imshow("test", img)
    cv2.imshow("test2", orig_image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

def test3():
    img = np.zeros((4,4,3), np.uint8)
    img[0][1] = 255
    img[1][0] = 255
    img[2][0] = 255
    img[3][0] = 255
    cols = img.shape[1]
    rows = img.shape[0]
    src = img
    for i in range(5):
        src = cv2.pyrUp(src, dstsize=(2 * src.shape[1], 2 * src.shape[0]))
    src = cv2.threshold(src, 1, 255, cv2.THRESH_BINARY)[1]
    new_img = cv2.resize(img, (200, 200), interpolation=cv2.INTER_LINEAR)
    new_img2 = cv2.resize(img, (200, 200), interpolation=cv2.INTER_CUBIC)
    new_img3 = cv2.resize(img, (200, 200), interpolation=cv2.INTER_LANCZOS4)
    new_img3 = cv2.threshold(new_img2, 10, 255, cv2.THRESH_BINARY)[1]
    new_img4 = cv2.threshold(new_img, 10, 255, cv2.THRESH_BINARY)[1]
    test_img = cv2.resize(img, (200, 200), interpolation=cv2.INTER_NEAREST)
    print(new_img3)
    cv2.imshow("test", new_img4)
    cv2.imshow("test2", new_img)
    cv2.imshow("test3", new_img2)
    cv2.imshow("test4", new_img3)
    cv2.imshow("src", src)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

def test4():
    sensor_readings = [0, 0, 0, 0, 0]
    IB_HEIGHT = 100
    GRAY = (105, 105, 105)
    info_bar = np.zeros((IB_HEIGHT, 500, 3), np.uint8)
    info_bar[:,:,:] = GRAY
    
    # draw sensor readings
    block_dim = 20
    block_spacing = 10
    block_y = 50
    start_x = 50
    for i, sr in enumerate(sensor_readings):
        if(sr == 0):
            color = (255, 100, 0)
        else:
            color = (0, 150, 255)
        block_x = start_x + i*(block_dim+block_spacing)
        top_left = (block_x-block_dim//2, block_y-block_dim//2)
        bot_right = (block_x+block_dim//2, block_y+block_dim//2)
        cv2.rectangle(info_bar, top_left, bot_right, color, -1)
    cv2.imshow("test2", info_bar)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
if __name__ == "__main__":
    test3()