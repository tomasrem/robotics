import socket
import math
from machine import Pin
from time import sleep

# --- LED and Button Setup ---
led_board = Pin(2, Pin.OUT)
led_yellow = Pin(4, Pin.OUT)
led_blue = Pin(23, Pin.OUT)
led_green = Pin(22, Pin.OUT)
led_red = Pin(21, Pin.OUT)
button_left = Pin(34, Pin.IN, Pin.PULL_DOWN)
button_right = Pin(35, Pin.IN, Pin.PULL_DOWN)


# --- Grid and Cost Definitions ---
grid = [
    [0, 1, 0, 1, 0, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
    [0, 1, 0, 1, 0, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0],
    [0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0],
    [0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 0, 1, 0, 1, 0],
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 0, 1, 0, 1, 0]
]

costs = [
     [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
     [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
     [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
     [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
     [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
     [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
     [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
     [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
     [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
     [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
     [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 10, 1],
     [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
     [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]
    ]

# --- Dijkstra Algorithm ---
def dijkstra(grid, costs, start, goal):
    rows, cols = len(grid), len(grid[0])
    visited = set()
    distances = {start: 0}
    parents = {start: None}
    queue = [(0, start)]
    directions = [(-1,0), (1,0), (0,-1), (0,1)]

    while queue:
        queue.sort()
        dist, node = queue.pop(0)
        if node in visited: continue
        visited.add(node)
        if node == goal: break

        for dx, dy in directions:
            neighbor = (node[0]+dy, node[1]+dx)
            if 0 <= neighbor[0] < rows and 0 <= neighbor[1] < cols and grid[neighbor[0]][neighbor[1]] == 0:
                new_dist = dist + costs[neighbor[0]][neighbor[1]]
                if neighbor not in distances or new_dist < distances[neighbor]:
                    distances[neighbor] = new_dist
                    parents[neighbor] = node
                    queue.append((new_dist, neighbor))

    path = []
    node = goal
    while node is not None:
        path.append(node)
        node = parents.get(node)
    return path[::-1]

# --- World to Grid Conversion ---
def world_to_grid(x, y):
    cell_sizex = 0.058823529
    cell_sizey = 0.061538461# meters
    origin_grid = (10, 16)  # robot at (x=0, y=0) → grid(10,16)
    row = origin_grid[0] - abs(int(round(y / cell_sizey)))
    col = origin_grid[1] - abs(int(round(x / cell_sizex)))
    return (row, col)

# --- Wait for Start ---
print("Click the button on the ESP32 to start the TCP server.")
while not button_left():
    sleep(0.25)
    led_board.value(not led_board())
led_board.off()

addr = socket.getaddrinfo('0.0.0.0', 1234)[0][-1]
server = socket.socket()
server.bind(addr)
server.listen(1)
print('Listening on', addr)

client, _ = server.accept()
print('Client connected!')

# --- Plan Path ---
start_node = (10, 16)
goal_node = (2, 0)
path = dijkstra(grid, costs, start_node, goal_node)
print("Planned path:", path)
current_path_index = 0

passed_intersection = False
current_state = 'forward'
counter = 0
COUNTER_MAX = 3
COUNTER_STOP = 50
COUNTER_MAX_2 = 10
sharp_turn = False
loop_count = 0 
# --- Main Loop ---
buffer = ""
while True:
    loop_count += 1
    mapturn = False
    state_updated = False  # reset flag every loop

    try:
        if client:
            data = client.recv(64).decode()
            buffer += data

            while '\n' in buffer:
                line, buffer = buffer.split('\n', 1)
                parts = line.strip().split(',')

                if len(parts) >= 4:
                    sensor_bits = parts[0].strip()
                    try:
                        x = int(parts[1].strip()) / 1000
                        y = int(parts[2].strip()) / 1000
                        phi = int(parts[3].strip()) / 1000
                    except ValueError:
                        print("Invalid coordinates:", parts)
                        continue
                    line_left = sensor_bits[0] == '1'
                    line_center = sensor_bits[1] == '1'
                    line_right = sensor_bits[2] == '1'

                    print(f"Sensor L/C/R: {sensor_bits} | x: {x} | y: {y} | phi: {phi}")               
                    gx, gy = world_to_grid(x, y) # returns grid from coordinates 
                    target = path[current_path_index] # passes the current path index to dijkstra algorithm
                    print(f"x: {x:.3f}, y: {y:.3f} → Grid: ({gx}, {gy}) | Target: {target}")
                    print(f"cpath:{current_path_index}")

                    if (gx, gy) == target:
                        print("✓ Reached", target)
                        current_path_index += 1
                        if current_path_index >= len(path):
                            print("✓ Goal reached.")
                            current_state = 'stop'
                            state_updated = True
                            continue
                            target = path[current_path_index]

                    # Determine direction
                    dx = target[1] - gy
                    dy = target[0] - gx
                    if  line_right == 0 and line_center == 0 and line_left == 0 :
                        passed_intersection = True 
                    
                    if dx == -1:
                        if passed_intersection == True :
                            current_state  = 'Sharpleft'
                        
                        else:
                            current_state  = 'forward'
                        state_updated = True
                        print('from map turn left')
                            
                 
                            
                    elif dx == 1:
                        if passed_intersection == True :
                            current_state  = 'Sharpright'
                            
                        else :
                            current_state  = 'forward'
                        state_updated = True
                        print('from map turn right')
            if loop_count > 10 :
                loop_count = 0
                passed_intersection = False 
                        
            # --- STATE MACHINE ---

            if current_state == 'forward':
                if line_right and not line_left:
                    current_state = 'turn_right'
                    state_updated = True
                    counter = 0
                elif line_left and not line_right:
                    current_state = 'turn_left'
                    state_updated = True
                    counter = 0
                    
                elif line_left and line_center and line_right:
                    current_state = 'turn_left'
                    state_updated = True
                    counter = 0
                        
                

            elif current_state == 'turn_right':
                counter += 1
              
                if counter >= COUNTER_MAX:
                    current_state = 'forward'
                    state_updated = True
                    counter = 0
                        
                        
  

            elif current_state == 'turn_left':
                counter += 1
                
                if counter >= COUNTER_MAX:
                    current_state = 'forward'
                    state_updated = True
                    counter = 0
  

            elif current_state == 'stop':
                led_board.on()
                counter += 1
                if counter >= COUNTER_STOP:
                    current_state = 'forward'
                    state_updated = True
                    counter = 0
                    led_board.off()

            if state_updated:
                client.send((current_state + '\n').encode())
                print(f"New state sent: {current_state} , {sharp_turn}")
                    
            sleep(0.05)

    except OSError:
        print("Connection lost. Waiting for reconnection.")
        client.close()
        client, _ = server.accept()
        print('Client reconnected.')
        current_path_index = 0






