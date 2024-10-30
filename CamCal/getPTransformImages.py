import numpy as np
import cv2 as cv
import pickle

cap = cv.VideoCapture(0)
cap.set(cv.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv.CAP_PROP_FRAME_HEIGHT, 1080)

num = 0

with open ('calibration.pkl', 'rb') as file:
    cameraMatrix, dist = pickle.load(file)

with open ('cameraMatrix.pkl', 'rb') as file:
    cameraMatrix = pickle.load(file)

with open ('dist.pkl', 'rb') as file:
    dist = pickle.load(file)

while cap.isOpened():
    ret, frame = cap.read()
    
    #Grab Frames and attempt to undistort
    h, w = frame.shape[:2]

    newCameraMatrix, roi = cv.getOptimalNewCameraMatrix(cameraMatrix, dist, (w,h), 1, (w,h))

    # Undistort with remapping
    mapx, mapy = cv.initUndistortRectifyMap(cameraMatrix, dist, None, newCameraMatrix, (w,h), 5)
    dst = cv.remap(frame, mapx, mapy, cv.INTER_LINEAR)

    # Crop the image
    x, y, w, h = roi
    dst = dst[y:y+h, x:x+w]
    cv.imshow('Undistorted', dst)

    k = cv.waitKey(5)

    if k == 27:
        break
    elif k == ord('s'): # wait for 's' key to save and exit
        cv.imwrite('reference/ref' + str(num) + '.png', dst, [int(cv.IMWRITE_PNG_COMPRESSION),0])
        print("image saved!")
        num += 1

    # End loop when hit d
    if cv.waitKey(20) & 0xFF==ord('d'):
        break

cap.release()

cv.destroyAllWindows()