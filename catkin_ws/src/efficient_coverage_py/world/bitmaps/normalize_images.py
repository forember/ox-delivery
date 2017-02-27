#!/usr/bin/env python

import glob
import os
import os.path

import cv2
import numpy as np
from PIL import Image


def main():
    os.chdir(os.path.realpath(os.path.dirname(__file__)))
    for filename in glob.iglob('*.png'):
        im = Image.open(filename).convert('RGB')
        im.save(filename)
        bordered = Image.new('RGB', (im.size[0] + 20, im.size[1] + 20))
        bordered.paste(im, (10, 10))
        bordered_filename = os.path.join('bordered', filename)
        bordered.save(bordered_filename)
        bordered_img = cv2.imread(bordered_filename)
        kernel = np.ones((9,9), np.uint8)
        expansion = cv2.erode(bordered_img, kernel, iterations=1)
        expanded_filename1 = os.path.join('bordered/expanded', '.' + filename)
        expanded_filename2 = os.path.join('bordered/expanded', filename)
        cv2.imwrite(expanded_filename1, expansion)
        (Image.open(expanded_filename1)
              .convert('1', dither=Image.NONE)
              .convert('RGB')
              .save(expanded_filename2))
        os.remove(expanded_filename1)


if __name__ == '__main__':
    main()
