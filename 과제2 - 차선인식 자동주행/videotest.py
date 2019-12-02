import time
import cv2
from linedetector import LineDetector

cap = cv2.VideoCapture('2.avi')
detector = LineDetector('', ros_node=False)

while True:
    ret, frame = cap.read()
    if not ret:
        break
    if cv2.waitKey(1) & 0xff == 27:
        break

    cv2.imshow('original', frame)
    detector.conv_image(frame)
    detector.detect_line()
    l, r = detector.direction_info
    detector.show_image()

cap.release()
cv2.destroyAllWindows()
