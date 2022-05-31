import cv2
import numpy as np
"""
tutorials: https://docs.opencv.org/4.x/d9/df8/tutorial_root.html
"""

def line_detect(image):
    line_de = cv2.imwr

def sense(image):
    # range of opencv H channel is 0 to 180 (half of 0 to 360)
<<<<<<< HEAD
    BLUE_START = np.array([100, 0, 0])
=======
    BLUE_START = np.array([90, 50, 0])
>>>>>>> 8c341b8f5f2843e19fc59306f17467841cd3b1bc
    BLUE_END = np.array([150, 255, 255])

    
    # convert image from BGR to HSV
    image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # mask out blue parts
    mask = cv2.inRange(image, BLUE_START, BLUE_END)
    return mask

def sample(mask, size, num_samples):
    mask = mask / 255
    width = mask.shape[1]
    height_val = mask.shape[0] // 2
    center = width // 2
    spacing = width // (num_samples + 1)
    positions = [center + i*spacing for i in range(-1*(num_samples // 2), num_samples // 2 + 1, 1)]
    readings = []
    for p in positions:
        sensed_area = mask[height_val-size//2:height_val+size//2+1,p-size//2:p+size//2+1]
        readings.append(np.average(sensed_area) > 0.5)

    return readings, positions

def test():
    # load image 
<<<<<<< HEAD
    image = cv2.imread("line_sensing\lp14.PNG")
=======
    image = cv2.imread("lp23.PNG")
>>>>>>> 8c341b8f5f2843e19fc59306f17467841cd3b1bc
    print(image.shape)
    # resize for easier visualization
    new_size = image.shape[:2]
    new_size = (new_size[1] // 4, new_size[0] // 4)
    image = cv2.resize(image, new_size)

    # get mask of blue parts
    line_mask = sense(image)

    # sample to get sensor readings
    SIZE = 25 # side length of sampled squares in pixels
    NUM_SAMPLES = 5
    readings, positions = sample(line_mask, SIZE, NUM_SAMPLES)

    height_val = image.shape[0]//2
    for p in positions:
        cv2.rectangle(image, (p-SIZE//2, height_val-SIZE//2), (p+SIZE//2, height_val+SIZE//2), (0, 0, 255), 5)
    
    # display mask
    print("Readings:", readings)
    cv2.imshow("orig", image)
    cv2.imshow("test", line_mask)

    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    test()
