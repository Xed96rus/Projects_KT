import cv2
import numpy as np

def findArucoMarkers(img, markerSize=6, totalMarkers=250, draw=True):
    imgGray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    key = getattr(cv2.aruco, f'DICT_{markerSize}X{markerSize}_{totalMarkers}')
    arucoDict = cv2.aruco.getPredefinedDictionary(key)
    arucoParam = cv2.aruco.DetectorParameters()
    corners, ids, rejected = cv2.aruco.detectMarkers(imgGray, arucoDict, parameters=arucoParam)
    if draw:
        cv2.aruco.drawDetectedMarkers(img, corners, ids)
    return img, ids, corners

cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    frame, ids, corners = findArucoMarkers(frame)
    print(ids, corners)
    cv2.imshow('Aruco Markers', frame)
    if cv2.waitKey(10) == 27: #Выход по клавише Esc
        break

cap.release()
cv2.destroyAllWindows() 