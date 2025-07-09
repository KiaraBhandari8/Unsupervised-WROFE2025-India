import os
import cv2

img= cv2.imread(os.path.join('.', 'draw.jpg'))

print(img.shape) 

cv2.imshow('img', img)
cv2.waitKey(0)