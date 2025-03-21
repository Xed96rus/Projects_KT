import cv2
import numpy as np
import matplotlib.pyplot as plt

# Загрузка изображения
image = cv2.imread("route.jpg", cv2.IMREAD_GRAYSCALE)

# Бинаризация (пороговое преобразование)
_, binary = cv2.threshold(image, 128, 255, cv2.THRESH_BINARY_INV)

# Отображаем бинаризованное изображение, чтобы проверить результат
plt.figure(figsize=(10, 5))
plt.subplot(1, 2, 1)
plt.imshow(binary, cmap="gray")
plt.title("Бинарное изображение")

# Скелетизация маршрута
# Используем морфологические операции для получения скелета
kernel = np.ones((3, 3), np.uint8)  # Ядро для морфологии (можно увеличить для лучшего эффекта)
skeleton = cv2.morphologyEx(binary, cv2.MORPH_HITMISS, kernel)

# Проверим, что скелетизация дала результат
plt.subplot(1, 2, 2)
plt.imshow(skeleton, cmap="gray")
plt.title("Скелетизация маршрута")

plt.show()