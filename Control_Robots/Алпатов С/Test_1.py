import numpy as np
from controller import Robot, Motor, PositionSensor

# Параметры робота
TIME_STEP = 32  # Время шага
radius = 0.063  # Радиус колеса (м)
base = 0.1826   # Расстояние до центра робота (м)

# Создание робота
robot = Robot()

# Получение моторов
motors = [
    robot.getDevice('wheel0_joint'),
    robot.getDevice('wheel1_joint'),
    robot.getDevice('wheel2_joint')
]

# Установка бесконечного режима работы моторов
for motor in motors:
    motor.setPosition(float('inf'))
    motor.setVelocity(0)

# Получение энкодеров
encoders = [
    robot.getDevice('wheel0_joint_sensor'),
    robot.getDevice('wheel1_joint_sensor'),
    robot.getDevice('wheel2_joint_sensor')
]

for encoder in encoders:
    encoder.enable(TIME_STEP)

# Матрица обратного преобразования (для получения Vx, Vy, W)
inv_matrix = (radius / 3) * np.array([
    [1, 1, -1],
    [0, np.sqrt(3), np.sqrt(3)],
    [1/(3*base), 1/(3*base), 1/(3*base)]
])

# Задание маршрута (список точек)
waypoints = [(2.0, 3.0), (1.0, 2.0), (3.0, 1.0)]  # Укажите координаты точек

# Начальные координаты
current_X, current_Y, current_theta = 0.0, 0.0, 0.0

# Временная переменная для предыдущих значений энкодеров
prev_enc_values = [0, 0, 0]

# Коэффициенты управления
Kp = 0.1  # Коэффициент движения
Kp_rotation = 1  # Коэффициент поворота

# Целевая точка
current_target = 0
target_X, target_Y = waypoints[current_target]
rotating = False  # Флаг поворота

while robot.step(TIME_STEP) != -1:
    # Считывание текущих значений энкодеров (в радианах)
    enc_values = [enc.getValue() for enc in encoders]

    # Вычисление изменения угла колеса
    d_enc = [enc_values[i] - prev_enc_values[i] for i in range(3)]
    prev_enc_values = enc_values  # Обновляем предыдущее значение

    # Вычисление скоростей колес (рад/с)
    wheel_speeds = np.array(d_enc) / (TIME_STEP / 1000.0)

    # Вычисление Vx, Vy, W
    Vx, Vy, omega = np.dot(inv_matrix, wheel_speeds)

    # Обновление положения робота
    dt = TIME_STEP / 1000.0  # Перевод в секунды
    current_X += Vx * dt
    current_Y += Vy * dt
    current_theta += omega * dt

    # Ограничение угла от -pi до pi
    current_theta = np.arctan2(np.sin(current_theta), np.cos(current_theta))

    if not rotating:
        # Вычисление ошибки координат
        error_X = target_X - current_X
        error_Y = target_Y - current_Y

        # Преобразуем глобальные ошибки в локальные
        V_x_local = np.cos(current_theta) * error_X + np.sin(current_theta) * error_Y
        V_y_local = -np.sin(current_theta) * error_X + np.cos(current_theta) * error_Y

        # Управление движением
        V_x = Kp * V_x_local
        V_y = Kp * V_y_local

        # Ограничение скорости
        V_x = np.clip(V_x, -1.0, 1.0)
        V_y = np.clip(V_y, -1.0, 1.0)

        # Преобразование (Vx, Vy, 0) в скорости колес
        motor_speeds = np.linalg.pinv(inv_matrix) @ np.array([V_x, V_y, 0])

        # Проверка на достижение цели
        if np.sqrt(error_X**2 + error_Y**2) < 0.05:
            print(f"Робот достиг точки {current_target + 1}: ({target_X}, {target_Y})")
            rotating = True  # Начинаем поворот
            target_theta = current_theta + np.radians(60)  # Увеличиваем угол на 60°
            target_theta = np.arctan2(np.sin(target_theta), np.cos(target_theta))  # Ограничение угла
    else:
        # Управление поворотом
        error_theta = target_theta - current_theta
        error_theta = np.arctan2(np.sin(error_theta), np.cos(error_theta))  # Ограничение угла
        omega = Kp_rotation * error_theta

        # Ограничение скорости вращения
        omega = np.clip(omega, -1.0, 1.0)

        # Преобразование (0, 0, omega) в скорости колес
        motor_speeds = np.linalg.pinv(inv_matrix) @ np.array([0, 0, omega])

        # Проверка завершения поворота
        if abs(error_theta) < 0.05:
            print(f"Робот повернулся на 60 градусов.")
            rotating = False  # Завершаем поворот
            current_target += 1  # Переход к следующей точке
            if current_target < len(waypoints):
                target_X, target_Y = waypoints[current_target]
            else:
                print("Маршрут завершен.")
                break

    # Установка скоростей моторов
    for i in range(3):
        motors[i].setVelocity(motor_speeds[i])

    # Вывод координат в консоль
    print(f"X: {current_X:.3f}, Y: {current_Y:.3f}, θ: {np.degrees(current_theta):.2f}°")