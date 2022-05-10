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

        degraded = self.degrade_line(self.image, 2, deg_type=1)
        dirt = self.simulate_dirt_and_grass(degraded, 0.01, 5, 6, area_type=0, material=0)
        grass = self.simulate_dirt_and_grass(self.image, 0.001, 5, 60, area_type=0, material=1)
        self.image = dirt | grass

        self.image = cv2.cvtColor(self.image, cv2.COLOR_GRAY2BGR)


    def initialize_env(self, image):
        image = ~image
        image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        return image

    """
    degrade line color
    image: grayscale image [0, 255]
    age: amount to degrade line
    deg_type: 0 - uniform degredation, 1 - degrade edges more than center
    returns: image with values degraded, elements in [0, 255]
    """
    def degrade_line(self, image, age, deg_type=0):
        dst = cv2.distanceTransform(image, cv2.DIST_L2, 5)

        center = np.where(dst > np.max(dst) / 2, image, 0)
        edges = np.where((dst <= np.max(dst) / 2) & (dst > 0), image, 0)

        deg_amt = 1 / age
        if(deg_type == 0):
            center, edges = center*deg_amt, edges*deg_amt
        elif(deg_type == 1):
            center, edges = center*(deg_amt*2), edges*deg_amt
        result = center + edges
        result = result.astype(np.uint8)

        return result

    """
    simulate dirt covering the line
    image: white/black line mask with no noise
    density: density of dirt centers, float [0, 1]
    area_type: 0 - uniform area in range [area_1, area_2], 1 - normally distributed area with mean area_1 and std_dev area_2
    material: 0 - dirt, 1 - grass
    """
    def simulate_dirt_and_grass(self, image, density, area_1, area_2, area_type=0, material=0):
        result = np.zeros(image.shape, np.uint8)
        if(material == 0):
            line_color = 0
            line_mask = np.where(image != 0, image, 0)
        elif(material == 1):
            line_color = 255
            line_mask = np.where(image == 0, 255, 0)
        line_mask = line_mask.astype(np.uint8)
        
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
                cv2.ellipse(result, (x, y), (major_axis, minor_axis), angle, 0, 360, 255, -1)
            except Exception as e:
                print(e)
                print(area, major_axis, minor_axis)
                return image

        result = np.where(line_mask != 0, result, 0)
        if(material == 0):
            result = np.where(result != 0, line_color, image)
        
        return result

