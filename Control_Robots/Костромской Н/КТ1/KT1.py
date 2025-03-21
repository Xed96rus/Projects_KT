from controller import Robot, Motor, PositionSensor
import math
# Объявляем глобальные переменные
# Константы
# Коэффициенты для линейного движения
# Коэффициенты для линейного движения

WHEEL_RADIUS = 0.05  # Радиус колеса
ROBOT_RADIUS = 0.15  # Расстояние от центра робота до колеса
TOLERANCE = 0.05  # Допустимая погрешность для достижения цели
ENCODER_UNIT = (2 * math.pi) / 159.23  # Коэффициент преобразования энкодера в радианы

robot = Robot()
timestep = 64

wheel0_motor = robot.getDevice('wheel0_joint')# Инициализация моторов
wheel1_motor = robot.getDevice('wheel1_joint')
wheel2_motor = robot.getDevice('wheel2_joint')

wheel0_motor.setPosition(float('inf'))  # Бесконечное вращение
wheel1_motor.setPosition(float('inf'))
wheel2_motor.setPosition(float('inf'))

wheel0_motor.setVelocity(0.0)  # Начальная скорость
wheel1_motor.setVelocity(0.0)
wheel2_motor.setVelocity(0.0)


wheel0_encoder = robot.getDevice('wheel0_joint_sensor')# Инициализация энкодеров
wheel1_encoder = robot.getDevice('wheel1_joint_sensor')
wheel2_encoder = robot.getDevice('wheel2_joint_sensor')

wheel0_encoder.enable(timestep)  # Включение энкодера
wheel1_encoder.enable(timestep)
wheel2_encoder.enable(timestep)


while robot.step(timestep) != -1:# Начальные значения энкодеров

    initial_wheel0_encoder = wheel0_encoder.getValue()
    initial_wheel1_encoder = wheel1_encoder.getValue()
    initial_wheel2_encoder = wheel2_encoder.getValue()
    break
def get_current_position():#Получить текущие координаты и ориентацию робота на основе данных энкодеров
    

    """Получить текущие координаты и ориентацию робота на основе данных энкодеров."""
    global initial_wheel0_encoder, initial_wheel1_encoder, initial_wheel2_encoder

    # Получение текущих значений энкодеров
    wheel0_encoder_value = wheel0_encoder.getValue() - initial_wheel0_encoder
    wheel1_encoder_value = wheel1_encoder.getValue() - initial_wheel1_encoder
    wheel2_encoder_value = wheel2_encoder.getValue() - initial_wheel2_encoder

    # Преобразование значений энкодеров в радианы
    wheel0_angle = wheel0_encoder_value * ENCODER_UNIT
    wheel1_angle = wheel1_encoder_value * ENCODER_UNIT
    wheel2_angle = wheel2_encoder_value * ENCODER_UNIT

    # Расчет перемещения робота
    delta_x = (wheel1_angle - wheel0_angle) * WHEEL_RADIUS / math.sqrt(3)
    delta_y = (2 * wheel2_angle - wheel0_angle - wheel1_angle) * WHEEL_RADIUS / 3
    delta_theta = (wheel0_angle + wheel1_angle + wheel2_angle) * WHEEL_RADIUS / (3 * ROBOT_RADIUS)


    return delta_x, delta_y, delta_theta
def reset_encoders():
    """Обнуление значений энкодеров."""
    global initial_wheel0_encoder, initial_wheel1_encoder, initial_wheel2_encoder  # Объявляем глобальные переменные
    initial_wheel0_encoder = wheel0_encoder.getValue()
    initial_wheel1_encoder = wheel1_encoder.getValue()
    initial_wheel2_encoder = wheel2_encoder.getValue()

def calculate_wheel_speeds(vx, vy, omega):#Рассчитать скорости колес на основе линейной и угловой скорости
    
  
    wheel0_speed = (vx - vy - omega * ROBOT_RADIUS) / WHEEL_RADIUS
    wheel1_speed = (vx + vy - omega * ROBOT_RADIUS) / WHEEL_RADIUS
    wheel2_speed = (-vx + vy - omega * ROBOT_RADIUS) / WHEEL_RADIUS
    return wheel0_speed, wheel1_speed, wheel2_speed
MAX_INTEGRAL = 0.1  # Максимальное значение интегральной составляющей

def pid_controller(current, target, kp, ki, kd, prev_error, integral):
    """PID-регулятор с ограничением интегральной составляющей."""
    error = target - current
    integral += error
    integral = max(min(integral, MAX_INTEGRAL), -MAX_INTEGRAL)  # Ограничение
    derivative = error - prev_error
    output = kp * error + ki * integral + kd * derivative
    return output, error, integral

def reached_target(current, target, tolerance):
    """Проверка, достигнута ли цель."""
    return abs(target - current) < tolerance

def reached_target(current, target, tolerance):
    """Проверка, достигнута ли цель."""
    return abs(target - current) < tolerance
def move_to_target(target_x, target_y, target_theta, kp_linear_x, ki_linear_x, kd_linear_x, kp_linear_y, ki_linear_y, kd_linear_y, kp_angular, ki_angular, kd_angular):
    """Движение к целевой точке с разными коэффициентами PID для движения по X и Y."""
    global initial_wheel0_encoder, initial_wheel1_encoder, initial_wheel2_encoder

    current_x, current_y, current_theta = 0.0, 0.0, 0.0

    # Переменные для PID-регулятора
    prev_error_x, prev_error_y, prev_error_theta = 0.0, 0.0, 0.0
    integral_x, integral_y, integral_theta = 0.0, 0.0, 0.0

    # Движение к координате X
    while robot.step(timestep) != -1:
        delta_x, delta_y, delta_theta = get_current_position()
        current_x += delta_x
        current_y += delta_y
        current_theta += delta_theta

        print(f"Текущее положение: X={current_x:.2f}, Y={current_y:.2f}, θ={current_theta:.2f}")

        if reached_target(current_x, target_x, TOLERANCE):
            print("Координата X достигнута!")
            break

        # Линейная скорость 0.5 м/с
        vx, prev_error_x, integral_x = pid_controller(current_x, target_x, kp_linear_x, ki_linear_x, kd_linear_x, prev_error_x, integral_x)
        vx = max(min(vx, 0.5), -0.5)  # Ограничение скорости до 0.5 м/с
        vy = 0  # Не двигаемся по Y
        omega = 0  # Не поворачиваем

        wheel0_speed, wheel1_speed, wheel2_speed = calculate_wheel_speeds(vx, vy, omega)
        wheel0_motor.setVelocity(-wheel0_speed)
        wheel1_motor.setVelocity(wheel1_speed)
        wheel2_motor.setVelocity(0)

    # Обнуление энкодеров после движения по X
    reset_encoders()

    # Движение к координате Y
    while robot.step(timestep) != -1:
        delta_x, delta_y, delta_theta = get_current_position()
        current_x += delta_x
        current_y += delta_y
        current_theta += delta_theta

        print(f"Текущее положение: X={current_x:.2f}, Y={current_y:.2f}, θ={current_theta:.2f}")

        if reached_target(current_y, target_y, TOLERANCE):
            print("Координата Y достигнута!")
            break

        # Линейная скорость 0.5 м/с
        vy, prev_error_y, integral_y = pid_controller(current_y, target_y, kp_linear_y, ki_linear_y, kd_linear_y, prev_error_y, integral_y)
       # vy = max(min(vy, 0.5), -0.5)  # Ограничение скорости до 0.5 м/с
        vx = 0  # Не двигаемся по X
        omega = 0  # Не поворачиваем

        wheel0_speed, wheel1_speed, wheel2_speed = calculate_wheel_speeds(vx, vy, omega)
        wheel0_motor.setVelocity(0)
        wheel1_motor.setVelocity(-wheel1_speed)
        wheel2_motor.setVelocity(wheel2_speed)

    # Обнуление энкодеров после движения по Y
    reset_encoders()
        # Поворот в нулевое положение (θ = 0)
    print("Поворот в нулевое положение...")
    while robot.step(timestep) != -1:
        delta_x, delta_y, delta_theta = get_current_position()
        current_x += delta_x
        current_y += delta_y
        current_theta += delta_theta

        print(f"Текущее положение: X={current_x:.2f}, Y={current_y:.2f}, θ={current_theta:.2f}")

        if reached_target(current_theta, 0, TOLERANCE):
            print("Робот вернулся в нулевое положение!")
            break

        omega, prev_error_theta, integral_theta = pid_controller(current_theta, 0, kp_angular, ki_angular, kd_angular, prev_error_theta, integral_theta)
        wheel0_speed, wheel1_speed, wheel2_speed = calculate_wheel_speeds(0, 0, omega)
        wheel0_motor.setVelocity(omega)
        wheel1_motor.setVelocity(omega)
        wheel2_motor.setVelocity(omega)

    # Обнуление энкодеров после поворота в нулевое положение
    reset_encoders()


    # Поворот на угол θ
    while robot.step(timestep) != -1:
        delta_x, delta_y, delta_theta = get_current_position()
        current_x += delta_x
        current_y += delta_y
        current_theta += delta_theta

        print(f"Текущее положение: X={current_x:.2f}, Y={current_y:.2f}, θ={current_theta:.2f}")

        if reached_target(current_theta, target_theta, TOLERANCE):
            wheel0_motor.setVelocity(0)
            wheel1_motor.setVelocity(0)
            wheel2_motor.setVelocity(0)
            print("Угол θ достигнут!")
            break

        omega, prev_error_theta, integral_theta = pid_controller(current_theta, target_theta, kp_angular, ki_angular, kd_angular, prev_error_theta, integral_theta)
        wheel0_speed, wheel1_speed, wheel2_speed = calculate_wheel_speeds(0, 0, omega)
        wheel0_motor.setVelocity(omega)
        wheel1_motor.setVelocity(omega)
        wheel2_motor.setVelocity(omega)

    # Обнуление энкодеров после поворота
    reset_encoders()

def main():
    # Координаты цели
    target_x = 1.0  # Координата X
    target_y = -1.0  # Координата Y
    target_theta = 1 # Угол θ (в радианах)

    # Коэффициенты PID для движения по X
    kp_linear_x = 0.5 # Увеличенный KP для быстрого реагирования
    ki_linear_x = 0.00000000000001 # Уменьшенный KI для избежания накопления ошибки
    kd_linear_x = 1  # Оставлен для сглаживания

    # Коэффициенты PID для движения по Y
    kp_linear_y = 0.24  # Немного меньше, чем для X, чтобы избежать перерегулирования
    ki_linear_y = 0.1  # Уменьшенный KI
    kd_linear_y = 1 # Оставлен для сглаживания
    # Коэффициенты PID для поворота на угол θ
    kp_angular = 0.5
    ki_angular = 0.001
    kd_angular = 0.05

    print("Движение к первой точке...")
    move_to_target(target_x, target_y, target_theta, kp_linear_x, ki_linear_x, kd_linear_x, kp_linear_y, ki_linear_y, kd_linear_y, kp_angular, ki_angular, kd_angular)
    print("Первая точка достигнута!")
    # Вторая точка
    target_x2 = -2.0  # Координата X второй цели
    target_y2 = 1.0  # Координата Y второй цели
    target_theta2 = 1  # Угол θ второй цели (в радианах)
    print("Движение ко второй точке...")
    move_to_target(target_x2, target_y2, target_theta2, kp_linear_x, ki_linear_x, kd_linear_x, kp_linear_y, ki_linear_y, kd_linear_y, kp_angular, ki_angular, kd_angular)
    print("Вторая точка достигнута!")

if __name__ == "__main__":
    main()
