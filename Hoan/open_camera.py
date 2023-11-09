import cv2

cap = cv2.VideoCapture(0)

while True:
    _, image = cap.read()

    cv2.imshow('Vieo', image)

    if cv2.waitKey(1) == ord('q'):
        break
