import cv2

cap = cv2.VideoCapture(0)

num = 0

cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

w = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
h = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)

print('resolution is: ', w,'x', h)
print('cam initialised')

while cap.isOpened():

    succes, img = cap.read() 

    k = cv2.waitKey(5)

    if k == 27:
        break
    elif k == ord('s'): # wait for 's' key to save and exit
        cv2.imwrite('images/img' + str(num) + '.png', img, [int(cv2.IMWRITE_PNG_COMPRESSION),0])
        print("image saved!")
        num += 1

    cv2.imshow('Img', img)

# Release and destroy all windows before termination
cap.release()

cv2.destroyAllWindows()