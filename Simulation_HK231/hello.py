import cv2

path = 'Hoan/butterfly.jpg'

img = cv2.imread(path)

cv2.imshow('tai anh',img)
# cv2.imwrite('basic/anh moi.png',img)

cv2.waitKey(2000)