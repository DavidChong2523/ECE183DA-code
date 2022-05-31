import cv2
import numpy as np

from sim.constants import *


class LineEnv():
    """
    image stored as BGR
    """
    def __init__(self, fname, noise_params=None):
        self.image = cv2.imread(fname)
        self.start, self.end = self.identify_endpoints(self.image)
        
        self.image = self.initialize_env(self.image)
        self.line = self.image
        self.error_map = self.generate_error_map(self.line)
        self.straight_dst, self.curve_dst = self.detect_straight(self.line)
        

        """
        noise_params: 
        {
            "dirt_prob": float,
            "dirt_a1": float,
            "dirt_a2": float,
            "dirt_area": {0, 1},
            "grass_prob": float,
            "grass_a1": float,
            "grass_a2": float,
            "grass_area": {0, 1}
        }
        """
        #degraded = self.degrade_line(self.image, 2, deg_type=1)
        degraded = self.image
        """
        dirt = self.simulate_dirt_and_grass(degraded, 
                                            noise_params["dirt_prob"], 
                                            noise_params["dirt_a1"], 
                                            noise_params["dirt_a2"], 
                                            area_type=noise_params["dirt_area"], 
                                            material=0)
        grass = self.simulate_dirt_and_grass(self.image, 
                                             noise_params["grass_prob"], 
                                             noise_params["grass_a1"], 
                                             noise_params["grass_a2"], 
                                             area_type=noise_params["grass_area"], 
                                             material=1)
        """
        dirt = self.simulate_error(degraded, noise_params["dirt_prob"], material=0)
        grass = self.simulate_error(degraded, noise_params["grass_prob"], material=1)
        self.image = dirt | grass
        
        self.image = cv2.cvtColor(self.image, cv2.COLOR_GRAY2BGR)


    def initialize_env(self, image):
        image = ~image
        image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        return image

    """
    get mask of straight sections of lines and curved section
    """
    def detect_straight(self, image):
        KERNEL_LEN = 25
        horizontalStructure = cv2.getStructuringElement(cv2.MORPH_RECT, (KERNEL_LEN, 1))
        horizontal = cv2.erode(image, horizontalStructure)
        horizontal = cv2.dilate(horizontal, horizontalStructure)
        curved = cv2.bitwise_xor(horizontal, image)

        # invert colors - line is black, space is white
        horizontal, curved = ~horizontal, ~curved
        straight_dst = cv2.distanceTransform(horizontal, cv2.DIST_L2, 5)
        curve_dst = cv2.distanceTransform(curved, cv2.DIST_L2, 5)
        return straight_dst, curve_dst

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

    """
    simulate dirt covering the line
    image: white/black line mask with no noise
    density: density of dirt centers, float [0, 1]
    material: 0 - dirt, 1 - grass
    """
    def simulate_error(self, image, density, material=0):
        if(material == 0):
            mask = np.where(image != 0, image, 0)
            background = image
        elif(material == 1):
            mask = np.where(image == 0, 255, 0).astype(np.uint8)
            background = np.zeros(image.shape, np.uint8)

        """
        binomial distribution with n=1 equivalent to bernoulli distribution
        https://stackoverflow.com/questions/47012474/bernoulli-random-number-generator
        """
        error_vals = np.random.binomial(size=image.shape, n=1, p=density)
        error_vals = np.where(error_vals != 0, 255, 0).astype(np.uint8)
        error_map = mask & error_vals

        result = cv2.bitwise_xor(error_map, background)
        return result 

    def generate_error_map(self, line):
        """
        https://answers.opencv.org/question/163561/looking-for-a-thinningskeletonizing-algorithm-with-opencv-in-python/#:~:text=You%20can%20simply%20pip%20install%20contrib%20with%3A%20pip,this%3A%20image%20%3D%20cv2.imread%28%22opencv.png%22%29%20thinned%20%3D%20cv2.ximgproc.thinning%28cv2.cvtColor%28image%2C%20cv2.COLOR_RGB2GRAY%29%29
        """
        dst = cv2.distanceTransform(line, cv2.DIST_L2, 5)
        vals = sorted(np.unique(dst))
        center = np.where(dst >= np.floor(vals[-2]), 255, 0).astype(np.uint8)
        #center = cv2.ximgproc.thinning(line)
        error_map = cv2.distanceTransform(~center, cv2.DIST_L2, 5)
        return error_map
        error_map = error_map / np.max(error_map) * 255
        error_map = np.where(error_map == 0, 1, 0)
        error_map = error_map.astype(np.uint8)*255
        return error_map

    """
    line: output of generate_error_map
    return: start position in (x, y) form
    """
    def identify_endpoints(self, line):
        line = cv2.cvtColor(line, cv2.COLOR_BGR2HSV)
        threshold = cv2.inRange(line, np.array([30, 0, 0]), np.array([90, 255, 255]))
        pts = np.where(threshold != 0)
        start = np.array([pts[1][0], pts[0][0]])
        end = np.array([pts[1][-1], pts[0][-1]])
        return start, end
        