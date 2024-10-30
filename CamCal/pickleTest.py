import numpy as np
import cv2 as cv
import glob
import pickle


with open ('calibration.pkl', 'rb') as file:
        cameraMatrix, dist = pickle.load(file)

with open ('cameraMatrix.pkl', 'rb') as file:
    cameraMatrix = pickle.load(file)

with open ('dist.pkl', 'rb') as file:
    dist = pickle.load(file)

print(f"\ncameraMatrix: {cameraMatrix}\n\n\n\n dist: {dist}")