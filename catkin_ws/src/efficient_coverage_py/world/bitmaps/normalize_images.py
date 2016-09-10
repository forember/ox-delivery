#!/usr/bin/env python

import glob
import os
import os.path

from PIL import Image


def main():
    os.chdir(os.path.realpath(os.path.dirname(__file__)))
    for filename in glob.iglob('*.png'):
        im = Image.open(filename).convert('RGB')
        im.save(filename)
        bordered = Image.new('RGB', (im.size[0] + 20, im.size[1] + 20))
        bordered.paste(im, (10, 10))
        bordered.save(os.path.join('bordered', filename))


if __name__ == '__main__':
    main()
