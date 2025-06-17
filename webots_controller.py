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


SHARP_TURN_DURATION =  5 # number of control loop steps
sharp_turn_counter = 0
sharp_turn_active = False
active_sharp_turn = None
sharp_turn_cooldown = 0
SHARP_TURN_COOLDOWN = 15


MAX_SPEED = 6.28
speed =  0.4* MAX_SPEED
R = 0.0205        # Wheel radius [m]
D = 0.052         # Distance between wheels [m]

# Initial robot pose
x = 0   # [m]
y = 0    # [m]
phi = 1.5  # [rad]



# PID constants
Kp = 1.3
Ki = 0.0
Kd = 0.25

# PID state variables
integral = 0.0
last_error = 0.0
error_history = []

# PID compute function
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
delta_t = timestep / 1000.0  # [s]
states = ['forward', 'turn_right', 'turn_left', 'stop', 'Sharpleft','Sharpright']
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
        print(data)   
    except socket.timeout:
        pass
            
    if current_state not in ['Sharpright', 'Sharpleft']: 
    
        # Calculate weighted error: left = -1, center = 0, right = +1
        weights = [-1, 0, 1]
        weighted_sum = 0
        total = 0
    
    
    
        binary = []
        if current_state == 'forward':
            binary = [0,1,0]
        elif current_state == 'turn_right':
            binary = [1,1,0]
        elif current_state == 'turn_left':
            binary = [0,1,1]
        elif current_state == 'stop':
            pass
    
    
    
    
    
    
    
        # received data and do the pid controll of the motor speeds
        for i in range(3):
            weighted_sum += weights[i] * binary[i]
            total += binary[i]
    
        # Determine error or recovery
        if total > 0:
            error = weighted_sum / total
            line_lost = False
        else:
            error = -1.5 if last_error < 0 else 1.5  # gently turn back
            line_lost = True
    
        # Compute PID correction
        correction = compute_pid(error, delta_t)
    
        # Adjust speed
        base_speed = 0.2 * MAX_SPEED if line_lost else 0.35 * MAX_SPEED
        left_speed = base_speed - correction
        right_speed = base_speed + correction
    
        # Clamp speeds
        left_speed = max(min(left_speed, MAX_SPEED), -MAX_SPEED)
        right_speed = max(min(right_speed, MAX_SPEED), -MAX_SPEED)
        
    # Sharp turn handling
    if sharp_turn_active:
        sharp_turn_counter += 1
    
        # Perform active sharp turn
        if active_sharp_turn == 'Sharpleft':
            left_speed = -0.2 * MAX_SPEED
            right_speed = 0.4 * MAX_SPEED
        elif active_sharp_turn == 'Sharpright':
            left_speed = 0.4 * MAX_SPEED
            right_speed = -0.2 * MAX_SPEED
    
        if sharp_turn_counter >= SHARP_TURN_DURATION:
            sharp_turn_active = False
            sharp_turn_counter = 0
            active_sharp_turn = None
    
    elif current_state in ['Sharpleft', 'Sharpright'] and not sharp_turn_active:
        # Start a new sharp turn
        sharp_turn_active = True
        active_sharp_turn = current_state
        sharp_turn_counter = 0
    
        if active_sharp_turn == 'Sharpleft':
            left_speed = -0.2 * MAX_SPEED
            right_speed = 0.4 * MAX_SPEED
        elif active_sharp_turn == 'Sharpright':
            left_speed = 0.4 * MAX_SPEED
            right_speed = -0.2 * MAX_SPEED
     
    

    # Apply to motors
    leftMotor.setVelocity(left_speed)
    rightMotor.setVelocity(right_speed)






   
    print(f'Sensor message: {message.strip()} - Current state: {current_state}')
    print(f"Encoders: L={encoderValues[0]:.3f}, R={encoderValues[1]:.3f}")
sock.close()