import numpy as np
import cv2

cap = cv2.VideoCapture('../../Datasets/Telefono/Calibration/Telefono/Video1.mp4')


while(cap.isOpened()):
    ret, frame = cap.read()

    #gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    if (type(frame) != type(None)):
        cv2.imshow('frame',frame)
        print(frame.shape[::-1])
        if cv2.waitKey(30) & 0xFF == ord('q'):
            break
    else:
        print("Finished")
        break

cap.release()
cv2.destroyAllWindows()