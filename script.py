import cv2

bear_min_color = (0, 66, 18)
bear_max_color = (13, 253, 149)

#cam = cv2.VideoCapture(0)

"""
while True:
    success, img = cam.read()

    if success:
        hsv = cv2.cvtColor(bi, cv2.COLOR_BGR2HSV)
        bear = cv2.inRange(hsv, bear_min_color, bear_min_color)
        cv2.imshow("image.jpg", bear)
        cv2.waitKey(50)"""

frame = cv2.imread("images/bear.jpg")
frame = cv2.resize(frame, (600, 800))

hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

binary_frame = cv2.inRange(hsv_frame, bear_min_color, bear_max_color)

cv2.imshow("TEST", binary_frame)
cv2.waitKey(0)
