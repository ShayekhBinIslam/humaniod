import cv2 
import numpy as np

# imgp = 'img/0.84.png'
imgp = 'img/39.44.png'

img = cv2.imread(imgp)
img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

print(img.shape, img)
l, w = img.shape
# consider only lower half of the image
img[0:int(l*0.75), :] = 0
img[img < 180] = 0
# img[img >= 180] = 255

cv2.imwrite('0.84.png', img)

img[img >= 180] = 1
white_px = img.sum()
total_px = np.prod(img.shape)
print(white_px/total_px)
