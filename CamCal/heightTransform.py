import time
import json
import argparse
from datetime import datetime
import math
import pickle

import cv2 as cv
import numpy as np


"""
The goal of this script is to find the height transform
matrix for the robot and save it in a pickle file
for later use in the video.py script. 

The original code takes in the following arguments
- Camera
- Width of the frame
- Height of the frame
- The other calibrations (already handled in video.py)
- The mouse input for clicking on the robot
- The reference image

"""

refimage = None

heightTransformMatrix = None
markerCenterPx = None
savedLow = []
savedHigh = []

cap = cv.VideoCapture(0)
cap.set(cv.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv.CAP_PROP_FRAME_HEIGHT, 1080)

with open ('calibration.pkl', 'rb') as file:
    cameraMatrix, dist = pickle.load(file)

with open ('cameraMatrix.pkl', 'rb') as file:
    cameraMatrix = pickle.load(file)

with open ('dist.pkl', 'rb') as file:
    dist = pickle.load(file)

with open ('transformMatrix.pkl', 'rb') as file:
    transformMatrix = pickle.load(file)

with open ('dimensions.pkl', 'rb') as file:
    dimensions = pickle.load(file)


def addPt(pt):
  if len(savedLow) == len(savedHigh):
    savedLow.append(pt)
  else:
    savedHigh.append(pt)
  print('Saved corner', len(savedLow), len(savedHigh))

def onmouse(event,x,y,flags,param):
  if event == cv.EVENT_LBUTTONDOWN:
    addPt([x, y])

cv.namedWindow('frame')
if args['mouse']:
  cv.setMouseCallback('frame', onmouse)


while cap.isOpened():
    ret, frame = cap.read()

    #Grab Frames and attempt to undistort
    resolution = frame.shape[:2]
    h, w = resolution

    newCameraMatrix, roi = cv.getOptimalNewCameraMatrix(cameraMatrix, dist, (w,h), 1, (w,h))

    # Undistort with remapping
    mapx, mapy = cv.initUndistortRectifyMap(cameraMatrix, dist, None, newCameraMatrix, (w,h), 5)
    dst = cv.remap(frame, mapx, mapy, cv.INTER_LINEAR)

    # Crop the image
    x, y, w, h = roi
    dst = dst[y:y+h, x:x+w]

    imgOutput = cv.warpPerspective(dst,transformMatrix,(dimensions[0],dimensions[1]))

    cv.imshow('Undistorted', imgOutput)


    markerCorners, markerIds = camera.detectAruco()

    if len(markerCorners) > 0:
        markerCornerSet = markerCorners[0]
        markerCornerSet = cv.undistortPoints(markerCornerSet, camera.cameraMatrix, camera.distCoeffs, P=camera.newCameraMatrix)
        markerCenterPx = (markerCornerSet[0][0] + markerCornerSet[0][1] + markerCornerSet[0][2] + markerCornerSet[0][3]) / 4.0

    if heightTransformMatrix is not None:
        transformedHigh = u.transformPixels(savedHigh, heightTransformMatrix)
    for i in range(len(transformedHigh)):
        o = savedHigh[i]
        t = transformedHigh[i]
        cv.line(frame, u.tup(o), u.tup(t), (200,200,200))
        cv.circle(frame, u.tup(o), 2, (200,200,200), -1)
        cv.circle(frame, u.tup(t), 2, (255,255,0), -1)

    if markerCenterPx is not None:
        markerCenterPxT = u.transformPixels([markerCenterPx], heightTransformMatrix)[0]
        cv.line(frame, u.tup(markerCenterPx), u.tup(markerCenterPxT), (255,255,255))
        cv.circle(frame, u.tup(markerCenterPx), 2, (255,255,255), -1)
        cv.circle(frame, u.tup(markerCenterPxT), 2, (255,0,255), -1)
    else:
        for pt in savedLow:
            cv.circle(frame, u.tup(pt), 2, (0,255,255), -1)
        for pt in savedHigh:
            cv.circle(frame, u.tup(pt), 2, (255,255,255), -1)
        if markerCenterPx is not None:
            cv.circle(frame, u.tup(markerCenterPx), 2, (255, 255, 0))

    if refimage is not None:
        cv.addWeighted(refimage, 0.25, frame, 1 - 0.25, 0, frame)

    status = str(len(savedLow)) + " low / " + str(len(savedHigh)) + " high"
    cv.putText(frame, status, (20, 20), cv.FONT_HERSHEY_PLAIN, 1, (255, 255, 255))
    status = "space: save, g: compute, s: save, c: clear, q: quit"
    cv.putText(frame, status, (20, 40), cv.FONT_HERSHEY_PLAIN, 1, (255, 255, 255))

    cv.imshow('frame', frame)
    key = cv.waitKey(1)

    if key & 0xFF == ord('s') and heightTransformMatrix is not None:
        u.saveJSON(args['calibrations'], u.videoFilename(cameraID, '-height'), {
            'heightTransformMatrix': heightTransformMatrix.tolist()
        })

    if key & 0xFF == ord(' ') and markerCenterPx is not None:
        addPt(markerCenterPx)

    if key & 0xFF == ord('g'):
        heightTransformMatrix, _ = cv.findHomography(np.array(savedHigh, np.float32), np.array(savedLow, np.float32), cv.LMEDS)
        camera.setHeight(heightTransformMatrix)

    if key & 0xFF == ord('c'):
        heightTransformMatrix = None
        camera.setPerspective(None)
        savedLow = []
        savedHigh = []

    p, f, frameidx = fps.update()

    if p:
        print("fps", f, "with any height?", heightTransformMatrix is not None)


    # End loop when hit d
    if cv.waitKey(20) & 0xFF==ord('q'):
        break

cap.release()

cv.destroyAllWindows()