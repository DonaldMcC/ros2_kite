
import numpy as np
import cv2

cap = cv2.VideoCapture(r'/home/donald/ros2_ws/src/ros2_kite/ros2_kite/choppedkite_horizshort.mp4')

cv2.startWindowThread()
cv2.namedWindow('contours')
fps = 15

while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()

    # Our operations on the frame come here
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Display the resulting frame
    cv2.imshow('contours', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()