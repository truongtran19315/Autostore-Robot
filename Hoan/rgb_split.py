import cv2

img = cv2.imread('Hoan/RGB_color_model.png', 1)

B, G, R = cv2.split(img)

cv2.imshow('RED', R)
cv2.imshow('BLUE', B)
cv2.imshow('GREEN', G)
cv2.imshow('anh goc', img)

cv2.waitKey()