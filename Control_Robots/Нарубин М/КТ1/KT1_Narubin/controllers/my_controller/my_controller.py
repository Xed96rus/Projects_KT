from controller import Robot
import numpy as np
# create the Robot instance.
robot = Robot()
# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())
MAX_SPEED = 137.46 # Радиан/с
R_WHEELS = 0.125
L_BASE = 0.182
motor_right = robot.getDevice('wheel2_joint')
motor_back = robot.getDevice('wheel0_joint')
motor_left = robot.getDevice('wheel1_joint')
motor_right.setPosition(float('inf'))
motor_back.setPosition(float('inf'))
motor_left.setPosition(float('inf'))
motor_right.setVelocity(0)
motor_back.setVelocity(0)
motor_left.setVelocity(0)
enc_right = robot.getDevice('wheel2_joint_sensor')
enc_back = robot.getDevice('wheel0_joint_sensor')
enc_left = robot.getDevice('wheel1_joint_sensor')
enc_right.enable(timestep)
enc_back.enable(timestep)
enc_left.enable(timestep)
encoders = [enc_right.getValue(),
            enc_back.getValue(),
            enc_left.getValue()]
start = [0] * 3

def update_enc(update = False, print_val = False):
    global start
    global encoders
    if not hasattr(update_enc, 'counter'):
        update_enc.counter = 0
    if (update == True):
        update_enc.counter = 0
    encoders = [enc_right.getValue(),
            enc_back.getValue(),
            enc_left.getValue()]
    if (update_enc.counter == 0):
        for i in range(3): start[i] = encoders[i]
        update_enc.counter = 1
    encoders = [enc_right.getValue() - start[0],
                enc_back.getValue() - start[1],
                enc_left.getValue() - start[2] ]
    if print_val:
        print(encoders) 
def speed_control(speed, update = False):
    if not hasattr(speed_control, 'counter'):
        speed_control.counter = 0
    if (update == True):
        speed_control.counter = 0
    if (speed_control.counter == 0):
        motor_back.setVelocity(speed[1])
        
        motor_right.setVelocity(speed[0])
        
        #0.512758
        motor_left.setVelocity(speed[2])
        
        speed_control.counter = 1
    else:
        kp = 1 #10
        kp2 = 1 #0.5
        motor_right.setVelocity(speed[0] * kp2)
        motor_back.setVelocity(speed[1] * kp)
        motor_left.setVelocity(speed[2] * kp)
# Матрица Якоби
J = np.array([[-np.sin(np.deg2rad(-60)), np.cos(np.deg2rad(-60)), L_BASE],
        [-np.sin(np.deg2rad(180)), np.cos(np.deg2rad(180)), L_BASE],
        [-np.sin(np.deg2rad(60)), np.cos(np.deg2rad(60)), L_BASE]])    

def wheels_speed(V_x, V_y, W):
    q = np.array([V_x, -V_y, W])
    return np.dot(J, q) / R_WHEELS

current_X = 0.0
current_Y = 0.0
current_theta = 0.0
def L_vector(target_X, target_Y):
    dx = (target_X - current_X)
    dy = (target_Y - current_Y)
    return np.sqrt(dx**2 + dy**2)
def Theta_1(target_X, target_Y):
    dx = (target_X - current_X)
    dy = (target_Y - current_Y)
    return np.arctan2(dy, dx)

def coords(target_X, target_Y, target_theta):
    global current_theta
    kp = 2
    first_theta = np.rad2deg(Theta_1(target_X, target_Y))
    first_theta = (first_theta * np.pi) / 180
    first_theta *= kp
    L = L_vector(target_X, target_Y)
    target_distance = L/R_WHEELS * 2.34
    first_move_distance = (L_BASE * first_theta) / R_WHEELS
    current_theta = np.rad2deg(first_theta)
    target_theta *= kp
    target_theta = ((target_theta - current_theta) * np.pi) / 180
    target_move_distance = (L_BASE * target_theta) / R_WHEELS
    #print(first_move_distance, target_distance, target_move_distance)
    return first_move_distance, target_distance, target_move_distance
    
kp = 1.8
angle = (np.deg2rad(90) * L_BASE) / R_WHEELS
step = 1
data_enc = 0
step = 1
speed = [0] * 3
#speed_control(speed)
motors = 0
while robot.step(timestep) != -1:
    n = 0
    update_enc()
    linear_enc = (abs(encoders[0]) + abs(encoders[2])) / 2
    angle_enc = abs(sum(np.array([encoders[x] for x in range(3)]) / 3))
    first_side, linear_move, target_side = coords(1, 0.5, 90)
    """if linear_move >= linear_enc:
        speed = wheels_speed(0, 1, 0)
        #speed[0] = 0
        speed_control(speed)
        print(speed)
    else:   
        speed_control([0, 0, 0])
        step = 3
        update_enc(True)"""
    if step == 1:
        print(f"Нужно повернуть:  {first_side}")
        print(f"Уже  повернули:   {angle_enc}")
        if first_side > 0:
            n = -1
        elif first_side < 0:
            n = 1
        if abs(first_side) >= angle_enc:
            speed = wheels_speed(0, 0, n * MAX_SPEED / 100)
            speed_control(speed)
        else:
            speed_control([0, 0, 0])
            step = 2
            update_enc(True)
        print(f"Скорость:         {[motor_right.getVelocity(),
           motor_back.getVelocity(),
           motor_left.getVelocity()]}")
    elif step == 2:
        print(f"Нужно проехать:  {linear_move}")
        print(f"Уже  проехали:   {linear_enc}")
        
        if linear_move >= linear_enc:
            speed = wheels_speed(MAX_SPEED / 100, 0, 0)
            speed_control(speed)
        else:   
            speed_control([0, 0, 0])
            step = 3
            update_enc(True)
        print(f"Скорость:         {[motor_right.getVelocity(),
           motor_back.getVelocity(),
           motor_left.getVelocity()]}")
    elif step == 3:
        print(f"Нужно повернуть:  {target_side}")
        print(f"Уже  повернули:   {angle_enc}")
        if target_side > 0:
            n = -1
        elif target_side < 0:
            n = 1
        if abs(target_side) >= angle_enc:
                speed = wheels_speed(0, 0, n * MAX_SPEED / 100)
                speed_control(speed)
        else:
            speed_control([0, 0, 0])
            step = 0
            update_enc(True)
        print(f"Скорость:         {[motor_right.getVelocity(),
           motor_back.getVelocity(),
           motor_left.getVelocity()]}")
    pass

# Enter here exit cleanup code.
