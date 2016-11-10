import cv2

#im2 = cv2.imread("test.WayGraph.CAC.In.1.png")
#im1 = cv2.imread("test.WayGraph.CAC.In.2.png")

#im3 = cv2.add(im1, im2)
im1 = cv2.imread("img0.png")
im = cv2.cvtColor(im1, cv2.COLOR_GRAY2RGB)

cv2.imwrite("img0.png", im)
