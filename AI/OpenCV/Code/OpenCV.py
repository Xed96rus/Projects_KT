import cv2
import numpy as np
import imutils
# Забрали словарь с параметрами, определяющим расссматриваемые ARUCO-маркеры
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)

# Генерация маркера
marker_id = 40
marker_image = cv2.aruco.generateImageMarker(dictionary=aruco_dict,
                                             id=marker_id,
                                             sidePixels=200,
                                             borderBits=1)
# Ручное добавление белой рамки
marker_with_border = np.ones(
    (200 + 2*50, 
     200 + 2*50), 
    dtype=np.uint8) * 255

# Вставляем маркер в центр
marker_with_border[
    50:-50,
    50:-50
] = marker_image
cv2.imwrite(filename='marker40.png', img=marker_with_border)
parameters = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(dictionary=aruco_dict,
                                   detectorParams=parameters)

image = cv2.imread("marker40.png")

#cv2_imshow(image) #выводим изображение в консоль, особенность Colab, не путать с cv2.imshow()

image = imutils.resize(image=image, width=1200)
cv2.imwrite(filename='marker_resize.png', img=image)

gray_image = cv2.cvtColor(src=image, code=cv2.COLOR_BGR2GRAY)
cv2.imwrite(filename='gray_image.png', img=gray_image)
gray_image = cv2.imread("gray_image.png")
corners, ids, rejected = detector.detectMarkers(image=gray_image)
if ids is not None:
  print("Углы маркеров", corners)
  print("ID маркеров:", ids)
  cv2.aruco.drawDetectedMarkers(gray_image, corners, ids)
  cv2.imshow("Detected", gray_image)
  cv2.waitKey(0)
else:
  print("Markers are not detected")