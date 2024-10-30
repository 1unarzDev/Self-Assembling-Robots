import numpy as np
import cv2 as cv
import time
import pickle

refNumber = 1

img = cv.imread('reference/ref' + str(refNumber) + '.png')

h, w = img.shape[:2]

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
	if len(corners) > 0:
		
		ids = ids.flatten()
		
		for (markerCorner, markerID) in zip(corners, ids):
			
			corners = markerCorner.reshape((4, 2))
			(topLeft, topRight, bottomRight, bottomLeft) = corners
			
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
			
			cv.putText(image, str(markerID),(topLeft[0], topLeft[1] - 10), cv.FONT_HERSHEY_SIMPLEX,
				0.5, (0, 255, 0), 2)
			
	return image


aruco_type = "DICT_6X6_250"
arucoDict = cv.aruco.getPredefinedDictionary(ARUCO_DICT[aruco_type])
arucoParams = cv.aruco.DetectorParameters()

width = 1000
height = int(width*(h/w))
img = cv.resize(img, (width, height), interpolation=cv.INTER_CUBIC)

corners, ids, rejected = cv.aruco.detectMarkers(img, arucoDict, parameters=arucoParams)

detected_markers = aruco_display(corners, ids, rejected, img)

# Aruco marker number starting from 0
a = 0

# Corner number starting from 0
b = 0

# Ignore the trash repetitive code below that came out of laziness to add an extra dimension to the matrix 
corner_dist_array0 = np.array([]).reshape(0,4)
corner_dist_array_temp0 = np.array([])

corner_dist_array1 = np.array([]).reshape(0,4)
corner_dist_array_temp1 = np.array([])

corner_dist_array2 = np.array([]).reshape(0,4)
corner_dist_array_temp2 = np.array([])

corner_dist_array3 = np.array([]).reshape(0,4)
corner_dist_array_temp3 = np.array([])

# 4 markers total each with 4 corners
for marker in corners:
	for point in corners[a][0]:
		distance0 = np.array([np.sqrt((point[0]**2)+(point[1]**2))])
		distance1 = np.array([np.sqrt(((w-point[0])**2)+(point[1]**2))])
		distance2 = np.array([np.sqrt((point[0]**2)+((h-point[1])**2))])
		distance3 = np.array([np.sqrt(((w-point[0])**2)+((h-point[1])**2))])

		corner_dist_array_temp0 = np.concatenate((corner_dist_array_temp0, distance0))
		corner_dist_array_temp1 = np.concatenate((corner_dist_array_temp1, distance1))
		corner_dist_array_temp2 = np.concatenate((corner_dist_array_temp2, distance2))
		corner_dist_array_temp3 = np.concatenate((corner_dist_array_temp3, distance3))

	corner_dist_array0 = np.vstack([corner_dist_array0, corner_dist_array_temp0])
	corner_dist_array1 = np.vstack([corner_dist_array1, corner_dist_array_temp1])
	corner_dist_array2 = np.vstack([corner_dist_array2, corner_dist_array_temp2])
	corner_dist_array3 = np.vstack([corner_dist_array3, corner_dist_array_temp3])

	corner_dist_array_temp0 = np.array([])
	corner_dist_array_temp1 = np.array([])
	corner_dist_array_temp2 = np.array([])
	corner_dist_array_temp3 = np.array([])
	
	a = a + 1

# Find the indices of the minimum value in the matrix
min_index0 = np.unravel_index(np.argmin(corner_dist_array0), corner_dist_array0.shape)
min_index1 = np.unravel_index(np.argmin(corner_dist_array1), corner_dist_array1.shape)
min_index2 = np.unravel_index(np.argmin(corner_dist_array2), corner_dist_array2.shape)
min_index3 = np.unravel_index(np.argmin(corner_dist_array3), corner_dist_array3.shape)

# Extract the row and column indices
row_index0, col_index0 = min_index0
row_index1, col_index1 = min_index1
row_index2, col_index2 = min_index2
row_index3, col_index3 = min_index3

board_height = int(max(corners[row_index2][0][col_index2][1],corners[row_index3][0][col_index3][1]) - min(corners[row_index0][0][col_index0][1], corners[row_index1][0][col_index1][1]))
board_width = int(max(corners[row_index1][0][col_index1][0], corners[row_index3][0][col_index3][0]) - min(corners[row_index0][0][col_index0][0], corners[row_index2][0][col_index2][0]))

pts1 = np.float32([corners[row_index0][0][col_index0],corners[row_index1][0][col_index1],corners[row_index2][0][col_index2],corners[row_index3][0][col_index3]])
pts2 = np.float32([[0,0],[board_width,0],[0,board_height],[board_width,board_height]])

transformMatrix = cv.getPerspectiveTransform(pts1,pts2)

# Save the transformMatrix result for later use
pickle.dump(transformMatrix, open( "transformMatrix.pkl", "wb" ))
pickle.dump([board_width, board_height], open( "dimensions.pkl", "wb"))

# May require fine tuning for proper size
imgOutput = cv.warpPerspective(img,transformMatrix,(board_width,board_height))

for x in range (0,4):
	cv.circle(img,(int(pts1[x][0]),int(pts1[x][1])),5,(0,0,255),cv.FILLED)
	
cv.imshow("Input", img)
cv.imshow("Output", imgOutput)

cv.imwrite("testresults.png", imgOutput)

cv.waitKey(0)