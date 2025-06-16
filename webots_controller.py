from controller import Robot
import socket
import numpy as np
import math
#-------------------------------------------------------
# Setup TCP socket communication to ESP32
HOST = '192.168.178.73'  # IP of ESP32
PORT = 1234

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect((HOST, PORT))
sock.settimeout(0.01)  # non-blocking receive

#-------------------------------------------------------
MAX_SPEED = 6.28
speed = 0.2 * MAX_SPEED
R = 0.0205        # Wheel radius [m]
D = 0.052         # Distance between wheels [m]


robot = Robot()
timestep = int(robot.getBasicTimeStep())
delta_t = timestep / 1000.0  # [s]
states = ['forward', 'turn_right', 'turn_left', 'stop']
current_state = 'forward'

# Initialize devices
ps = [robot.getDevice(f'ps{i}') for i in range(8)]
for s in ps:
    s.enable(timestep)

gs = [robot.getDevice(f'gs{i}') for i in range(3)]
for g in gs:
    g.enable(timestep)
    
encoder = []
encoderNames = ['left wheel sensor', 'right wheel sensor']
for i in range(2):
    encoder.append(robot.getDevice(encoderNames[i]))
    encoder[i].enable(timestep)
    

leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)

oldEncoderValues = []

# Initial robot pose
x = 0   # [m]
y = 0    # [m]
phi = 1.5  # [rad]

def get_wheels_speed(encoderValues, oldEncoderValues, delta_t):
    wl = (encoderValues[0] - oldEncoderValues[0]) / delta_t
    wr = (encoderValues[1] - oldEncoderValues[1]) / delta_t
    
    return [wl, wr]

def get_robot_speeds(wl, wr, r, d):
    u = r * (wr + wl) / 2
    w = r * (wr - wl) / d
    return [u, w]

def get_robot_pose(u, w, x_old, y_old, phi_old, delta_t):
    dx = u * math.cos(phi_old)
    dy = u * math.sin(phi_old)
    dphi = w

    x_new = x_old + dx * delta_t
    y_new = y_old + dy * delta_t
    phi_new = phi_old + dphi * delta_t
    phi_new = (phi_new + math.pi) % (2 * math.pi) - math.pi
    return [x_new, y_new, phi_new]

#-------------------------------------------------------
while robot.step(timestep) != -1:
    # --- SEE ---
    gsValues = [gs[i].getValue() for i in range(3)]

    line_right = gsValues[0] > 600
    line_center = gsValues[1] > 600
    line_left = gsValues[2] > 600
        
    encoderValues = [e.getValue() for e in encoder]
    
    if len(oldEncoderValues) == 0:
        oldEncoderValues = encoderValues
        continue  # skip first loop
    
    [wl, wr] = get_wheels_speed(encoderValues, oldEncoderValues, delta_t)
    [u, w] = get_robot_speeds(wl, wr, R, D)
    [x, y, phi] = get_robot_pose(u, w, x, y, phi, delta_t)
    
    oldEncoderValues = encoderValues
    xpos = int(x*1000)
    ypos = int(y*1000)
    phip = int(phi*1000)
    # Update encoder values
    oldEncoderValues = encoderValues
    
    # Compose message: sensor bits + left_encoder + right_encoder
    message = f"{'1' if line_left else '0'}{'1' if line_center else '0'}{'1' if line_right else '0'},{xpos},{ypos}, {phip}\n"
    
    # --- THINK ---
    try:
        sock.send(message.encode())
        data = sock.recv(32).decode().strip()
        if data in states:
            current_state = data
    except socket.timeout:
        pass

    if current_state == 'forward':
        leftSpeed = rightSpeed = speed
    elif current_state == 'turn_right':
        leftSpeed, rightSpeed = 0.5 * speed, -0.5 * speed
    elif current_state == 'turn_left':
        leftSpeed, rightSpeed = -0.5 * speed, 0.5 * speed
    elif current_state == 'stop':
        leftSpeed = rightSpeed = 0.0

    # --- ACT ---
    leftMotor.setVelocity(leftSpeed)
    rightMotor.setVelocity(rightSpeed)
    print(f'Sensor message: {message.strip()} - Current state: {current_state}')
    print(f"Encoders: L={encoderValues[0]:.3f}, R={encoderValues[1]:.3f}")
sock.close()