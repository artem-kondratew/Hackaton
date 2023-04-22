import cv2


camera = cv2.VideoCapture(0)


while True:
    success, frame = camera.read()

    if success:
        cv2.imshow("TON", cv2.flip(cv2.flip(frame, 0), 1))
        cv2.waitKey(20)
