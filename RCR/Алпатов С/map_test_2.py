import numpy as np
import scipy.interpolate as spi
import matplotlib.pyplot as plt

# Исходные точки маршрута
x = np.array([0, 2, 1, -1, 1, 5])
y = np.array([0, 2, 6, 1, -3, 0])

# Создание параметра t для равномерного распределения точек
t = np.linspace(0, 1, len(x))

# Интерполяция с использованием сглаживания (InterpolatedUnivariateSpline)
spl_x = spi.InterpolatedUnivariateSpline(t, x, k=2)  # Интерполируем x
spl_y = spi.InterpolatedUnivariateSpline(t, y, k=2)  # Интерполируем y

# Генерация новых точек
t_new = np.linspace(0, 1, 100)
x_smooth = spl_x(t_new)
y_smooth = spl_y(t_new)

# Закрепляем значения x[0] и y[0] на их местах
x_smooth[0] = x[0]
y_smooth[0] = y[0]

# Отображение исходных точек с прямыми линиями
#plt.plot(x, y, 'ro-', label="Исходные точки (с прямыми линиями)")
plt.plot(x, y, 'ro-', label="Исходные точки (с прямыми линиями)")
plt.legend()
plt.show()

# Отображение сглаженного маршрута
plt.plot(x_smooth, y_smooth, 'b-', label="Сглаженный маршрут")
plt.legend()
plt.show()