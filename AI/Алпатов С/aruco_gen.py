import cv2

# Определение словаря Aruco
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

# Генерация маркера (ID=0, размер 200x200 пикселей)
marker_id = 0
marker_size = 400
marker_image = cv2.aruco.generateImageMarker(aruco_dict, marker_id, marker_size)

# Сохранение изображения маркера
cv2.imwrite("aruco_marker.png", marker_image)

# Отображение маркера
cv2.imshow("Aruco Marker", marker_image)
cv2.waitKey(0)
cv2.destroyAllWindows()