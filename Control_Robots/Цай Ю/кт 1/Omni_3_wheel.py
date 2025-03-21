from controller import Supervisor
import math
import time

TIME_STEP = 64  

TARGET_POINTS = [
    (0, 3),    
    (4, 0.9),  
    (-2, 5),   
    (3, 2)     
]

WHEEL_RADIUS = 0.04  
ROBOT_RADIUS = 0.4   

DISTANCE_THRESHOLD = 0.01  

MAX_WHEEL_SPEED = 5.0 

supervisor = Supervisor()

robot_node = supervisor.getFromDef('ROBOT')  

if robot_node is None:
    raise ValueError("Робот с DEF 'ROBOT' не найден в сцене. Проверьте DEF-имя.")

translation_field = robot_node.getField('translation')  # Поле для координат
rotation_field = robot_node.getField('rotation')        # Поле для ориентации

left_wheel = supervisor.getDevice('wheel0_joint')  
right_wheel = supervisor.getDevice('wheel1_joint') 
rear_wheel = supervisor.getDevice('wheel2_joint')  

for wheel in [left_wheel, right_wheel, rear_wheel]:
    wheel.setPosition(float('inf'))  
    wheel.setVelocity(0.0)           

def clamp_value(value, min_value, max_value):
    # ограничиваем значение в заданном диапазоне
    return max(min(value, max_value), min_value)

def calculate_wheel_speeds(linear_velocity_x, linear_velocity_y, angular_velocity):
    # скорости колес на основе линейной и угловой скорости
    wheel_speeds = []
    for angle in [0, 2 * math.pi / 3, 4 * math.pi / 3]:  # Углы для трех колес
        speed = (-math.sin(angle) * linear_velocity_x + math.cos(angle) * linear_velocity_y + ROBOT_RADIUS * angular_velocity) / WHEEL_RADIUS
        wheel_speeds.append(clamp_value(speed, -MAX_WHEEL_SPEED, MAX_WHEEL_SPEED))
    return wheel_speeds

current_target_index = 0  # Индекс текущей цели
current_target = TARGET_POINTS[current_target_index]  # Текущая цель
is_target_reached = False  # Флаг достижения цели
is_returning_home = False  # Флаг возврата в нулевую точку
stop_timer = None          # Таймер для остановки

while supervisor.step(TIME_STEP) != -1:
  
    robot_position = translation_field.getSFVec3f()
    x, y = robot_position[0], robot_position[1]  

    # Получение текущего угла поворота (yaw)
    robot_rotation = rotation_field.getSFRotation()
    yaw = robot_rotation[3] 

    # Если цель достигнута, выводим сообщение и ждем 3 секунды
    if is_target_reached:
        if time.time() - stop_timer >= 3:  
            is_target_reached = False
            if is_returning_home:
                # После возврата в нулевую точку переходим к следующей цели
                is_returning_home = False
                current_target_index += 1
                if current_target_index < len(TARGET_POINTS):
                    current_target = TARGET_POINTS[current_target_index]
                    print(f"Возврат в нулевую точку завершен. Переход к цели {current_target_index + 1}.")
                else:
                    print("Все, куда смотришь")
                    break
            else:
                # После достижения цели начинаем возврат в нулевую точку
                is_returning_home = True
                current_target = (0, 0) 
                print(f"Цель {current_target_index + 1} достигнута! Возвращаемся в нулевую точку.")
        else:
            continue  

    # Если цель не достигнута, выводим текущие координаты
    if not is_target_reached:
        print(f"Текущие координаты: ({x:.2f}, {y:.2f})")
        
    # сюда надо
    target_x, target_y = current_target

    # разница до нужной координаты
    delta_x = target_x - x
    delta_y = target_y - y
    distance_to_target = math.hypot(delta_x, delta_y)

    if distance_to_target < DISTANCE_THRESHOLD:
        for wheel in [left_wheel, right_wheel, rear_wheel]:
            wheel.setVelocity(0.0)
        is_target_reached = True
        stop_timer = time.time()  
        continue

    target_angle = math.atan2(delta_y, delta_x)
    angle_difference = target_angle - yaw

    if angle_difference > math.pi:
        angle_difference -= 2 * math.pi
    elif angle_difference < -math.pi:
        angle_difference += 2 * math.pi

    forward_speed = 2.0  
    rotation_speed = angle_difference * 0.1  

    linear_velocity_x = forward_speed * math.cos(angle_difference)
    linear_velocity_y = forward_speed * math.sin(angle_difference)
    angular_velocity = rotation_speed

    wheel_speeds = calculate_wheel_speeds(linear_velocity_x, linear_velocity_y, angular_velocity)

    left_wheel.setVelocity(wheel_speeds[0])
    right_wheel.setVelocity(wheel_speeds[1])
    rear_wheel.setVelocity(wheel_speeds[2])