import cv2

camera = cv2.VideoCapture(0)

while True:
    cx = 0
    cy = 0
    success, frame = camera.read()

    if success:
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        binary = cv2.inRange(hsv, (76, 59, 59), (179, 255, 255))

        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        if len(contours) != 0:
            maxc = max(contours, key = cv2.contourArea)
            moments = cv2.moments(maxc)

            if moments["m00"] > 20:
                cx = int(moments["m10"] / moments["m00"])
                cy = int(moments["m01"] / moments["m00"])

                iSee = True


    cv2.imshow("Camera", binary)
    cv2.waitKeyEx(20)
