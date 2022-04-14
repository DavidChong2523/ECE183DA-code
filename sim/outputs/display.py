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
    vertices = [(0, 0), (length, 0)]
    vertices = [np.array([[v[0]], [v[1]]]) for v in vertices]
    vertices = [rotation_mat(angle) @ v for v in vertices]
    vertices = [v + center for v in vertices]
    return vertices

def draw_rect(image, dim, center, angle):
    vertices = rotated_rect(dim, center, angle)
    vertices = [(int(v[0][0]), int(v[1][0])) for v in vertices]
    color = (255, 0, 0)
    thickness = 2
    for i in range(4):
        cv2.line(image, vertices[i], vertices[(i+1)%4], color, thickness)
    return image

def draw_line(image, length, center, angle):
    vertices = rotated_line(length, center, angle)
    vertices = [(int(v[0][0]), int(v[1][0])) for v in vertices]
    color = (0, 255, 0)
    thickness = 1
    cv2.arrowedLine(image, vertices[0], vertices[1], color)
    return image
    
    

if __name__ == "__main__":
    image = np.zeros((500,500, 3))
    angle = 0
    image = draw_rect(image, (20, 40), (100, 100), angle)
    image = draw_line(image, 10, (100, 100), angle)
    cv2.imshow("test", image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()