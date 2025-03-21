from controller import Robot
import numpy as np

# Параметры платформы
R = 0.125  # Радиус колеса (м)
L = 0.1775  # Расстояние от центра до колеса (м)
alpha = np.array([np.pi / 6, 5 * np.pi / 6, 3 * np.pi / 2])  # Углы расположения колес

# Якобиан
J = np.array([
    [-np.sin(alpha[0]), np.cos(alpha[0]), L],
    [-np.sin(alpha[1]), np.cos(alpha[1]), L],
    [-np.sin(alpha[2]), np.cos(alpha[2]), L]
])

# Обратная матрица якобиана
J_inv = np.linalg.inv(J)

# Коэффициенты регулятора
k_p = 0.5  # Коэффициент для линейной скорости
k_theta = 1.0  # Коэффициент для угловой скорости

# Начальные и целевые значения
x_goal, y_goal, theta_goal = 10.0, 10.0, 0.0  # Целевое положение

# Инициализация робота
robot = Robot()
timestep = int(robot.getBasicTimeStep())

# Получение моторов
motors = []
for i in range(3):
    motor = robot.getDevice(f'wheel{i}_joint')
    motor.setPosition(float('inf'))
    motor.setVelocity(0.0)
    motors.append(motor)

# Получение GPS
gps = robot.getDevice('gps')
gps.enable(timestep)

# Получение компаса
compass = robot.getDevice('compass')
compass.enable(timestep)

# Основной цикл симуляции
while robot.step(timestep) != -1:
    # Получение текущего положения и ориентации
    position = gps.getValues()
    x_robot, y_robot = position[0], position[1]
    
    orientation = compass.getValues()
    theta_robot = np.arctan2(orientation[1], orientation[0])

    # Ошибка положения и ориентации
    e_x = x_goal - x_robot
    e_y = y_goal - y_robot
    e_theta = theta_goal - theta_robot

    # Желаемая скорость платформы
    v_x = k_p * e_x
    v_y = k_p * e_y
    omega = k_theta * e_theta

    # Скорости колес
    u = np.dot(J_inv, np.array([v_x, v_y, omega]))

    # Установка скоростей моторов
    for i in range(3):
        motors[i].setVelocity(u[i])

    # Проверка на достижение цели
    if np.sqrt(e_x**2 + e_y**2) < 0.1 and abs(e_theta) < 0.1:
        print("Цель достигнута!")
        break