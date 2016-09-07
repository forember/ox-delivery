#!/usr/bin/env python

WID = 0

def show_tour(tour, image_filename, window_id=None):
    global WID
    from PIL import Image, ImageDraw
    import cv2, numpy
    if window_id is None:
        window_id = WID
        WID += 1
    window_name = str(window_id)
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    pil_im = Image.open(image_filename).convert('RGB')
    draw = ImageDraw.Draw(pil_im)
    for i in range(1, len(tour)):
        x1, y1 = tour[i - 1]
        x2, y2 = tour[i]
        r = 4
        draw.ellipse([x1-r,y1-r,x1+r,y1+r], fill=(128, 128, 255))
        draw.ellipse([x2-r,y2-r,x2+r,y2+r], fill=(255, 0, 0))
        draw.line([x1,y1,x2,y2], fill=(0, 128, 0))
        cv2_im = cv2.cvtColor(numpy.array(pil_im), cv2.COLOR_RGB2BGR)
        cv2.imshow(window_name, cv2_im)
        while True:
            key = cv2.waitKey()
            if key == 10 or key == 32:
                break
            elif key == 27:
                cv2.destroyWindow(window_name)
                return
    cv2.destroyWindow(window_name)

def show_tour_lines(tour_file, image_filename):
    import json
    tours = []
    current_tour = None
    for line in tour_file:
        line = line.strip()
        if line.startswith('Start'):
            current_tour = []
        elif line.startswith('End'):
            tours.append(current_tour)
            current_tour = None
        elif line:
            line = line.replace(') (', '), (')
            line = '[' + line + ']'
            line = line.replace('(', '[').replace(')', ']')
            current_tour.extend(json.loads(line))
    for tour in tours:
        show_tour(tour, image_filename)

def main():
    import sys
    if len(sys.argv) != 3:
        print('usage: {} <tour file> <image file>'.format(sys.argv[0]))
        raise SystemExit(1)
    tour_filename = sys.argv[1]
    image_filename = sys.argv[2]
    with open(tour_filename) as tour_file:
        show_tour_lines(tour_file, image_filename)

if __name__ == '__main__':
    main()
