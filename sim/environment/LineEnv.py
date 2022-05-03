import cv2
import numpy as np

from sim.constants import *


class LineEnv():
    """
    image stored as BGR
    """
    def __init__(self, fname):
        self.image = cv2.imread(fname)
        self.image = self.initialize_env(self.image)
        self.image = self.degrade_line(self.image, 1, deg_type=1)
        self.image = self.simulate_dirt(self.image, 0.1, 5, 6, area_type=0)

        self.image = cv2.cvtColor(self.image, cv2.COLOR_GRAY2BGR)
        cv2.imshow("test", self.image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    def initialize_env(self, image):
        image = ~image
        image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        return image

    """
    degrade line color
    image: grayscale image
    age: amount to degrade line
    deg_type: 0 - uniform degredation, 1 - degrade edges more than center
    returns: image with values degraded, elements in [0, 255]
    """
    def degrade_line(self, image, age, deg_type=0):
        dst = cv2.distanceTransform(image, cv2.DIST_L2, 5)

        center = np.where(dst > np.max(dst) / 2, 1, 0)
        edges = np.where((dst <= np.max(dst) / 2) & (dst > 0), 1, 0)

        deg_amt = 1 / age
        if(deg_type == 0):
            center, edges = center*deg_amt, edges*deg_amt
        elif(deg_type == 1):
            center, edges = center*(deg_amt*2), edges*deg_amt
        result = center + edges
        result = (result*255).astype(np.uint8)

        return result

    """
    simulate dirt covering the line
    image: grayscale image
    density: density of dirt centers, float [0, 1]
    area_type: 0 - uniform area in range [area_1, area_2], 1 - normally distributed area with mean area_1 and std_dev area_2
    """
    def simulate_dirt(self, image, density, area_1, area_2, area_type=0):
        """
        binomial distribution with n=1 equivalent to bernoulli distribution
        https://stackoverflow.com/questions/47012474/bernoulli-random-number-generator
        """
        centers = np.random.binomial(size=image.shape, n=1, p=density)
        centerx, centery = np.nonzero(centers)
        
        for x, y in zip(centerx, centery):
            if(area_type == 0):
                area = np.random.uniform(low=area_1, high=area_2)
            elif(area_type == 1):
                area = np.random.normal(loc=area_1, scale=area_2)
            
            if(area <= 2):
                continue

            major_axis = np.random.uniform(low=2, high=area/2) 
            minor_axis = area / major_axis
            major_axis, minor_axis = int(major_axis / 2), int(minor_axis / 2)
            angle = np.random.uniform(0, 360)
            angle = int(angle)

            try:
                cv2.ellipse(image, (x, y), (major_axis, minor_axis), angle, 0, 360, 0, -1)
            except Exception as e:
                print(e)
                print(area, major_axis, minor_axis)
                return image
        return image

