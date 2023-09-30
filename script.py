import cv2

cam = cv2.VideoCapture(0)

while True:
    success, frame = cam.read()
    #frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    if success:
        cv2.imwrite("image.jpg", frame)
        cv2.waitKey(50)
