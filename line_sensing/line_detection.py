import cv2
import numpy as np
"""
tutorials: https://docs.opencv.org/4.x/d9/df8/tutorial_root.html
"""

def sense(image):
    # range of opencv H channel is 0 to 180 (half of 0 to 360)
    BLUE_START = np.array([100, 100, 0])
    BLUE_END = np.array([150, 255, 255])
    
    # convert image from BGR to HSV
    image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # mask out blue parts
    mask = cv2.inRange(image, BLUE_START, BLUE_END)
    return mask

def test():
    # load image 
    image = cv2.imread("ECE183DA-code\line_sensing\lp31.PNG")
    print(image.shape)
    # resize for easier visualization
    new_size = image.shape[:2]
    new_size = (new_size[0] // 4, new_size[1] // 4)
    image = cv2.resize(image, new_size)

    # get mask of blue parts
    line_mask = sense(image)

    # display mask
    cv2.imshow("orig", image)
    cv2.imshow("test", line_mask)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    test()
