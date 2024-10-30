import numpy as np
import cv2 as cv
import pickle
from ultralytics import YOLO

model = YOLO('best.pt')

with open ('calibration.pkl', 'rb') as file:
    cameraMatrix, calib_dist = pickle.load(file)

with open ('cameraMatrix.pkl', 'rb') as file:
    cameraMatrix = pickle.load(file)

with open ('dist.pkl', 'rb') as file:
    calib_dist = pickle.load(file)

with open ('transformMatrix.pkl', 'rb') as file:
    transformMatrix = pickle.load(file)

with open ('dimensions.pkl', 'rb') as file:
    dimensions = pickle.load(file)

ARUCO_DICT = {
	"DICT_4X4_50": cv.aruco.DICT_4X4_50,
	"DICT_4X4_100": cv.aruco.DICT_4X4_100,
	"DICT_4X4_250": cv.aruco.DICT_4X4_250,
	"DICT_4X4_1000": cv.aruco.DICT_4X4_1000,
	"DICT_5X5_50": cv.aruco.DICT_5X5_50,
	"DICT_5X5_100": cv.aruco.DICT_5X5_100,
	"DICT_5X5_250": cv.aruco.DICT_5X5_250,
	"DICT_5X5_1000": cv.aruco.DICT_5X5_1000,
	"DICT_6X6_50": cv.aruco.DICT_6X6_50,
	"DICT_6X6_100": cv.aruco.DICT_6X6_100,
	"DICT_6X6_250": cv.aruco.DICT_6X6_250,
	"DICT_6X6_1000": cv.aruco.DICT_6X6_1000,
	"DICT_7X7_50": cv.aruco.DICT_7X7_50,
	"DICT_7X7_100": cv.aruco.DICT_7X7_100,
	"DICT_7X7_250": cv.aruco.DICT_7X7_250,
	"DICT_7X7_1000": cv.aruco.DICT_7X7_1000,
	"DICT_ARUCO_ORIGINAL": cv.aruco.DICT_ARUCO_ORIGINAL,
	"DICT_APRILTAG_16h5": cv.aruco.DICT_APRILTAG_16h5,
	"DICT_APRILTAG_25h9": cv.aruco.DICT_APRILTAG_25h9,
	"DICT_APRILTAG_36h10": cv.aruco.DICT_APRILTAG_36h10,
	"DICT_APRILTAG_36h11": cv.aruco.DICT_APRILTAG_36h11
}

def aruco_display(corners, ids, rejected, image):
	centersX = np.array([])
	centersY = np.array([])
	angles = np.array([])
	
	if len(corners) > 0:
		
		ids = ids.flatten()

		for (markerCorner, markerID) in zip(corners, ids):
			
			corners = markerCorner.reshape((4, 2))
			(topLeft, topRight, bottomRight, bottomLeft) = corners
            
            # Calculate angle using the top side of the marker
			angle_rad = np.arctan2(topRight[1] - topLeft[1], topRight[0] - topLeft[0])
			angle_deg = np.degrees(angle_rad)
			angles = np.append(angles, angle_deg)
			
			topRight = (int(topRight[0]), int(topRight[1]))
			bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
			bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
			topLeft = (int(topLeft[0]), int(topLeft[1]))

			cv.line(image, topLeft, topRight, (0, 255, 0), 2)
			cv.line(image, topRight, bottomRight, (0, 255, 0), 2)
			cv.line(image, bottomRight, bottomLeft, (0, 255, 0), 2)
			cv.line(image, bottomLeft, topLeft, (0, 255, 0), 2)
			
			cX = int((topLeft[0] + bottomRight[0]) / 2.0)
			cY = int((topLeft[1] + bottomRight[1]) / 2.0)
			cv.circle(image, (cX, cY), 4, (0, 0, 255), -1)

			centersX = np.append(centersX, cX)
			centersY = np.append(centersY, cY)
      
			cv.putText(image, str(markerID),(topLeft[0], topLeft[1] - 10), cv.FONT_HERSHEY_SIMPLEX,
				0.5, (0, 255, 0), 2)  
	else:
		return None
			
	return image, centersX, centersY, angles

aruco_type = "DICT_6X6_250"
arucoDict = cv.aruco.getPredefinedDictionary(ARUCO_DICT[aruco_type])
arucoParams = cv.aruco.DetectorParameters()

def video_detection(cap):

	iteration = 0
	found = False

	while not found:
		ret, frame = cap.read()
		
		#Grab Frames and attempt to undistort
		h, w = frame.shape[:2]

		newCameraMatrix, roi = cv.getOptimalNewCameraMatrix(cameraMatrix, calib_dist, (w,h), 1, (w,h))

		# Undistort with remapping
		mapx, mapy = cv.initUndistortRectifyMap(cameraMatrix, calib_dist, None, newCameraMatrix, (w,h), 5)
		dst = cv.remap(frame, mapx, mapy, cv.INTER_LINEAR)

		# Crop the image
		x, y, w, h = roi
		dst = dst[y:y+h, x:x+w]

		width = 1000
		height = int(width*(h/w))
		dst = cv.resize(dst, (width, height), interpolation=cv.INTER_CUBIC)

		imgOutput = cv.warpPerspective(dst,transformMatrix,(dimensions[0],dimensions[1]))
		
		corners, ids, rejected = cv.aruco.detectMarkers(imgOutput, arucoDict, parameters=arucoParams)

		if len(corners) > 0:
			found = True

		if iteration == 1:
			print("MOVE THE ROBOT")

		iteration += 1

	if(iteration > 2):
		print("ROBOT DETECTED")
        
	detected_markers, centersX, centersY, angles = aruco_display(corners, ids, rejected, imgOutput)  

	results = model(source=imgOutput, conf=0.5, verbose=False)

	# Extract classes and masks
	classes = results[0].boxes.cls.tolist()
	obs = results[0].masks.xy if hasattr(results[0].masks, 'xy') else None
    
	new_h, new_w = imgOutput.shape[:2]

	return obs, centersX, centersY, corners, ids, angles, new_h, new_w