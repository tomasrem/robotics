from controller import Robot
import socket
import numpy as np
import math
import time

# Setup TCP socket communication to ESP32
HOST = '192.168.178.73'   # IP of ESP32
PORT = 1234

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect((HOST, PORT))
sock.settimeout(0.01)

turning = False
turn_start_angle = None
turn_direction = None
turn_start_time = 0
COOLDOWN_PERIOD = 8
ANGLE_THRESHOLD = 1.2

MAX_SPEED = 6.28
speed = 0.4 * MAX_SPEED
R = 0.0205
D = 0.052

x, y, phi = 0, 0, 1.5
Kp, Ki, Kd = 1.3, 0.0, 0.25
integral, last_error = 0.0, 0.0
error_history = []

def compute_pid(error, dt):
    global integral, last_error, error_history
    error_history.append(error)
    if len(error_history) > 5:
        error_history.pop(0)

    smoothed_error = sum(error_history) / len(error_history)
    integral += smoothed_error * dt
    derivative = (smoothed_error - last_error) / dt if dt > 0 else 0.0

    last_error = smoothed_error
    return Kp * smoothed_error + Ki * integral + Kd * derivative

robot = Robot()
timestep = int(robot.getBasicTimeStep())
delta_t = timestep / 1000.0
states = ['forward', 'turn_right', 'turn_left', 'stop', 'Sharpleft', 'Sharpright']
current_state = 'forward'

ps = [robot.getDevice(f'ps{i}') for i in range(8)]
for s in ps:
    s.enable(timestep)

gs = [robot.getDevice(f'gs{i}') for i in range(3)]
for g in gs:
    g.enable(timestep)

encoder = [robot.getDevice(name) for name in ['left wheel sensor', 'right wheel sensor']]
for e in encoder:
    e.enable(timestep)

leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)

oldEncoderValues = []

def get_wheels_speed(encoderValues, oldEncoderValues, delta_t):
    wl = (encoderValues[0] - oldEncoderValues[0]) / delta_t
    wr = (encoderValues[1] - oldEncoderValues[1]) / delta_t
    return wl, wr

def get_robot_speeds(wl, wr, r, d):
    u = r * (wr + wl) / 2
    w = r * (wr - wl) / d
    return u, w

def get_robot_pose(u, w, x_old, y_old, phi_old, delta_t):
    x_new = x_old + u * math.cos(phi_old) * delta_t
    y_new = y_old + u * math.sin(phi_old) * delta_t
    phi_new = phi_old + w * delta_t
    phi_new = (phi_new + math.pi) % (2 * math.pi) - math.pi
    return x_new, y_new, phi_new

while robot.step(timestep) != -1:
    gsValues = [g.getValue() for g in gs]
    line_right = gsValues[0] > 600
    line_center = gsValues[1] > 600
    line_left = gsValues[2] > 600

    encoderValues = [e.getValue() for e in encoder]

    if not oldEncoderValues:
        oldEncoderValues = encoderValues
        continue

    wl, wr = get_wheels_speed(encoderValues, oldEncoderValues, delta_t)
    u, w = get_robot_speeds(wl, wr, R, D)
    x, y, phi = get_robot_pose(u, w, x, y, phi, delta_t)

    oldEncoderValues = encoderValues

    xpos, ypos, phip = int(x*1000), int(y*1000), int(phi*1000)
    message = f"{int(line_left)}{int(line_center)}{int(line_right)},{xpos},{ypos},{phip}\n"

    try:
        sock.send(message.encode())
        data = sock.recv(32).decode().strip()
        if data in states:
            current_state = data
        print(f"Received data: {data}")
    except socket.timeout:
        pass

    current_time = time.time()

    if current_state in ['Sharpright', 'Sharpleft']:
        if not turning and (current_time - turn_start_time) > COOLDOWN_PERIOD:
            turning = True
            turn_start_angle = phi
            turn_direction = current_state
            turn_start_time = current_time
            print("Turning initiated")

    if turning:
        angle_diff = abs(phi - turn_start_angle)
        print(f"Angle difference: {angle_diff:.3f}")
        if angle_diff < ANGLE_THRESHOLD:
            if turn_direction == 'Sharpright':
                left_speed = 0.2 * MAX_SPEED
                right_speed = -0.2 * MAX_SPEED
            else:
                left_speed = -0.2 * MAX_SPEED
                right_speed = 0.2 * MAX_SPEED
        else:
            turning = False
            current_state = 'forward'
            left_speed = right_speed = 0
            print("Turn completed")
    else:
        binary = [0, 1, 0] if current_state == 'forward' else [1, 1, 0] if current_state == 'turn_right' else [0, 1, 1]

        if current_state == 'stop':
            left_speed = right_speed = 0
        else:
            weights = [-1, 0, 1]
            total = sum(binary)
            weighted_sum = sum(w * b for w, b in zip(weights, binary))
            error = weighted_sum / total if total else (-1.5 if last_error < 0 else 1.5)

            correction = compute_pid(error, delta_t)
            base_speed = 0.35 * MAX_SPEED
            left_speed = max(min(base_speed - correction, MAX_SPEED), -MAX_SPEED)
            right_speed = max(min(base_speed + correction, MAX_SPEED), -MAX_SPEED)

    leftMotor.setVelocity(left_speed)
    rightMotor.setVelocity(right_speed)

    print(f"Current state: {current_state}, Left speed: {left_speed:.2f}, Right speed: {right_speed:.2f}")
    print(message)
sock.close()