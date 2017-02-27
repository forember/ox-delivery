#!/usr/bin/env python

import cv2
import numpy as np
from PIL import Image

img = cv2.imread('Compare_1.map.png', 0)
img = cv2.copyMakeBorder(img, 1, 1, 1, 1,  cv2.BORDER_CONSTANT, value=0)
kernel = np.ones((9,9),np.uint8)
expansion = cv2.erode(img, kernel, iterations=2)
expansion = expansion[1:-1, 1:-1]
cv2.imwrite('Compare_1_expanded.map.png', expansion)
(Image.open('Compare_1_expanded.map.png')
        .convert('1', dither=Image.NONE)
        .convert('RGB')
        .save('Compare_1_expanded.map.png'))
