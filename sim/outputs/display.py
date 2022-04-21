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
    
    

if __name__ == "__main__":
    image = np.zeros((500,500, 3))
    for i in range(10):
        angle = 0
        angle = angle2image(0)
        pos = (10, 10)
        pos = real2image(pos, image.shape)
        pos = (pos[1], pos[0])
        image = draw_rect(image, (20, 40), pos, angle)
        image = draw_line(image, 10, pos, angle)
        cv2.imshow("test", image)
        cv2.waitKey(0)
    cv2.arrowedLine(image, (500, 0), (200, 400), (255, 255, 0))
    cv2.imshow("test2", image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
