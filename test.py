import numpy as np
import cv2 as cv
# Create a black image
img = np.zeros((8,8,3), np.uint8)

cv.ellipse(img,(4,4),(2,1),45,0,360,(255, 255, 0),-1)

img = cv.resize(img, (512, 512), interpolation=cv.INTER_NEAREST)
cv.imshow("test", img)
cv.waitKey(0)
cv.destroyAllWindows()