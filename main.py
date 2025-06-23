import heapq
import socket
import network
import time
from time import sleep

# --- Wi-Fi static IP setup ---
SSID       = 'Ziggo1541315'
PASSWORD   = 'pchcestzy3gfnvdV'
STATIC_IP  = '192.168.178.73'
SUBNET     = '255.255.255.0'
GATEWAY    = '192.168.178.1'
DNS        = '8.8.8.8'

def connect_wifi_static(ssid, password, ip, subnet, gateway, dns, timeout=15):
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    wlan.ifconfig((ip, subnet, gateway, dns))
    print('Static IP configured:', wlan.ifconfig())
    if not wlan.isconnected():
        print('Connecting to Wi-Fi…')
        wlan.connect(ssid, password)
        start = time.time()
        while not wlan.isconnected():
            if time.time() - start > timeout:
                raise RuntimeError('Wi-Fi connection timed out')
            print('.', end='')
            time.sleep(1)
        print()
    print('Connected, network config:', wlan.ifconfig())
    return wlan

connect_wifi_static(SSID, PASSWORD, STATIC_IP, SUBNET, GATEWAY, DNS)

# --- TCP server ---
addr   = socket.getaddrinfo('0.0.0.0', 1234)[0][-1]
server = socket.socket()
server.bind(addr)
server.listen(1)
print('Listening on', addr)
client, _ = server.accept()
print('Client connected!')

# --- Occupancy grid: 0=road, 1=wall ---
grid = [
    [0,1,0,1,0,1,0,1,1,1,1,1,1,1,1,1,1],
    [0,1,0,1,0,1,0,1,1,1,1,1,1,1,1,1,1],
    [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
    [0,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,0],
    [0,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0],
    [0,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,0],
    [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
    [0,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,0],
    [0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,0],
    [0,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,0],
    [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
    [1,1,1,1,1,1,1,1,1,1,0,1,0,1,0,1,0],
    [1,1,1,1,1,1,1,1,1,1,0,1,0,1,0,1,0],
]

costs = [
     [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 10],
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
ROWS = len(grid)
COLS = len(grid[0])

# --- Map scaling & origin ---
cell_sizex  = 0.058823529
cell_sizey  = 0.061538461
origin_col  = 16
origin_row  = 10

_last_free = None

def world_to_grid(x, y):
    """
    Convert (x,y)→(col,row). If result is a wall, search 3×3 neighbors
    for nearest road. If none, fall back to last_free.
    """
    global _last_free
    cf = origin_col + x / cell_sizex
    rf = origin_row - y / cell_sizey
    c = int(round(cf))
    r = int(round(rf))
    c = max(0, min(COLS-1, c))
    r = max(0, min(ROWS-1, r))

    if grid[r][c] == 1:
        best = None
        best_d2 = float('inf')
        for dr in (-1,0,1):
            for dc in (-1,0,1):
                nr, nc = r+dr, c+dc
                if 0 <= nr < ROWS and 0 <= nc < COLS and grid[nr][nc] == 0:
                    d2 = (cf-nc)**2 + (rf-nr)**2
                    if d2 < best_d2:
                        best_d2 = d2
                        best = (nc, nr)
        if best:
            c, r = best
        elif _last_free:
            return _last_free

    _last_free = (c, r)
    return (c, r)

# --- Named waypoints ---
points = {
    'A': ((10, 10), (-0.31,   0)),
    'B': ((12, 10), (-0.205,  0)),
    'C': ((14, 10), (-0.11,   0)),
    'D': ((16, 10), (0,       0)),
    'E': ((16,  6), (0,       0.24)),
    'F': ((16,  4), (0,       0.35)),
    'G': ((16,  2), (0,       0.5)),
    'H': (( 8,  2), (-0.5,    0.5)),
    'I': (( 8,  4), (-0.5,    0.35)),
    'J': (( 8,  6), (-0.5,    0.24)),
    'K': (( 8,  8), (-0.5,    0.145)),
    'L': (( 8, 10), (-0.5,    0)),
    'M': (( 0, 10), (-1,      0)),
    'N': (( 0,  8), (-1,      0.145)),
    'O': (( 0,  6), (-1,      0.24)),
    'P': (( 0,  2), (-1,      0.5)),
    'R': (( 2,  2), (-0.895,  0.5)),
    'S': (( 4,  2), (-0.795,  0.5)),
    'T': (( 6,  2), (-0.69,   0.5)),
}


class FindMatchingPoint:
    def __init__(self, points):
        self.points = points

    def __getitem__(self, gx_gy):
        gx, gy = gx_gy
        for label, ((pgx, pgy), (x, y)) in self.points.items():
            if pgx == gx and pgy == gy:
                return label, x, y
        raise KeyError(f"No matching point for gx={gx}, gy={gy}")
    
findmatchingpoint = FindMatchingPoint(points)

grid_to_waypoint = {
    loc: (lbl, tx, ty)
    for lbl, (loc, (tx, ty)) in points.items()
}

# --- Dijkstra pathfinding ---


def dijkstra(grid, costs, start, goal):
    visited = set()
    dist    = {start:0}
    parent  = {start:None}
    frontier= [(0,0,start)]
    moves   = [(0,-1),(-1,0),(0,1),(1,0)]
    while frontier:
        d,_,node = heapq.heappop(frontier)
        if node in visited: continue
        visited.add(node)
        if node == goal: break
        for pri, (dx,dy) in enumerate(moves):
            nb = (node[0]+dx, node[1]+dy)
            x,y = nb
            if not (0<=x<COLS and 0<=y<ROWS): continue
            if grid[y][x]==1: continue
            nd = d + costs[y][x]
            if nd < dist.get(nb, float('inf')):
                dist[nb], parent[nb] = nd, node
                heapq.heappush(frontier, (nd, pri, nb))
    path, cur = [], goal
    while cur is not None:
        path.append(cur)
        cur = parent[cur]
    return list(reversed(path))

# --- Plan path once ---
start_label, goal_label = 'D', 'O'
start_node = points[start_label][0]
goal_node  = points[goal_label][0]
full_path  = dijkstra(grid, costs, start_node, goal_node)
path       = full_path[1:] if full_path and full_path[0]==start_node else full_path
print("Planned path:", path)

# --- Loop state init ---
buffer        = ""
loop_count    = 0
passed_int    = False
in_int        = False
state         = 'forward'
counter       = 0
MAX_TURNS     = 5
STOP_PERIOD   = 20
cal_x_off     = 0.0
cal_y_off     = 0.0
cal_phi_off   = 0.0
idx           = 0
calkey = ''
delta = ''  
calibration = False
last_gx, last_gy = start_node
delay_after_turn = 2000  # 2 seconds
turn_event_time = None
delay_task_pending = False
last_direction = ''  # to store the direction after delay
prev_dir = 'R'
prev_turn = 'Left'
c_pos = ''
keyy = ''
gx = ''
gy = ''
lbl = None

def save_direction():
    global last_direction, direction, prev_dir
    last_direction = direction
    prev_dir = direction   # Update the robot's known facing direction after the turn
    print("Direction saved after delay as:", last_direction)
    print("Updated prev_dir to:", prev_dir)
    
while True:
    loop_count += 1
    state_updated = False

    data = client.recv(64).decode()
    buffer += data
    while '\n' in buffer:
        line, buffer = buffer.split('\n',1)
        parts = line.split(',')
        if len(parts)<4:
            continue

        bits = parts[0].strip()
        try:
            raw_x   = int(parts[1])/1000.0
            raw_y   = int(parts[2])/1000.0
            raw_phi = int(parts[3])/1000.0
        except ValueError:
            print("Invalid data:", parts)
            continue

        L,C,R = bits[0]=='1', bits[1]=='1', bits[2]=='1'

        # --- Intersection & waypoint calibration ---
        now_int = not (L or C or R)
        
        gx_raw, gy_raw = world_to_grid(raw_x, raw_y)
        keyy = (gx, gy)
      
    
        #############

        try:
            lbl, tx_true, ty_true = findmatchingpoint[keyy]
            print(f"**********            Matched: Label={lbl}, x={tx_true}, y={ty_true}             ***********")
        except KeyError:
            # No matching point found, handle accordingly
            pass



        #if the current position matches some waypoint and its not yet calibrated
        if lbl != None  and calibration == False:
            
            calkey = keyy
            calibration = True
            
        # still havent been calibrated ( its the first time )   
        if calibration == True :
            if now_int and not in_int:
                cal_x_off   = raw_x   - tx_true
                cal_y_off   = raw_y   - ty_true
                cal_phi_off = raw_phi
                # PRINT waypoint arrival, offsets, and true position
                print(f"Reached waypoint {lbl}")
                print(f"  → offsets: x_off={cal_x_off:.3f}, y_off={cal_y_off:.3f}, φ_off={cal_phi_off:.3f}")
                print(f"  → calibrated to true pos: ({tx_true:.3f},{ty_true:.3f})")
                lbl, x, y = None, None, None
            else:
                print ('waiting for intersection to calibrate ')
                #in_int = True
            
        if not now_int and in_int:
            in_int = False
            calibration = False

        # --- Apply calibration to pose ---
        x   = raw_x   - cal_x_off
        y   = raw_y   - cal_y_off
        phi = raw_phi - cal_phi_off
        print(raw_x,raw_y)
        print(x,y)
        # --- Raw grid cell from calibrated pose ---
        gx_raw, gy_raw = world_to_grid(x, y)

        # --- Determine current target ---
        if idx < len(path):
            tx, ty = path[idx]
        else:
            tx, ty = goal_node

        # --- AXIS LOCKING ---
        if tx != last_gx:
            gx, gy = gx_raw, last_gy
        elif ty != last_gy:
            gx, gy = last_gx, gy_raw
        else:
            #gx, gy = gx_raw, gy_raw
            pass

        last_gx, last_gy = gx, gy

        print(f"Grid: ({gx},{gy}) → target: ({tx},{ty})")

        # --- Advance on reaching target cell ---
        if gx==tx and gy==ty:
            if (gx,gy) not in grid_to_waypoint:
                print(f"✓ Reached grid cell ({gx},{gy})")
            idx += 1
            if idx>=len(path):
                state = 'stop'
            state_updated = True

        # --- FSM: sharp-turn & line-follow ---
        
        # determining current direction of robot 

        if prev_dir == 'U':
            if prev_turn == 'Left':
                c_pos = 'L'
            elif prev_turn == 'Right':
                c_pos = 'R'

        elif prev_dir == 'L':
            if prev_turn == 'Left':
                c_pos = 'D'
            elif prev_turn == 'Right':
                c_pos = 'U'

        elif prev_dir == 'R':
            if prev_turn == 'Left':
                c_pos = 'U'
            elif prev_turn == 'Right':
                c_pos = 'D'

        elif prev_dir == 'D':
            if prev_turn == 'Left':
                c_pos = 'R'
            elif prev_turn == 'Right':
                c_pos = 'L'

        direction = c_pos
        current_direction = direction  # update current direction for later saving

        # ---- Intersection detection ----
        if not (L or C or R):
            passed_int = True  # no line detected = passed intersection

        # ---- Determine which way to turn based on direction and goal ----
        delta = None

        if direction == 'U':
            if (tx - gx) <= -1:
                delta = 'Left'
            elif (tx - gx) >= 1:
                delta = 'Right'
                
            if (tx - gx) == 0:
                delta = 'None'
        elif direction == 'D':
            if (tx - gx) >= 1:
                delta = 'Left'
            elif (tx - gx) <= -1:
                delta = 'Right'
                 
            if (tx - gx) == 0:
                delta = 'None'    

        elif direction == 'L':
            if (ty - gy) <= -1:
                delta = 'Right'
            elif (ty - gy) >= 1:
                delta = 'Left'
                 
            if (ty - gy) == 0:
                delta = 'None'

        elif direction == 'R':
            if (ty - gy) >= 1:
                delta = 'Right'
            elif (ty - gy) <= -1:
                delta = 'Left'
                 
            if (ty - gy) == 0:
                delta = 'None'
        print("Direction:", direction)
        did_turn = False
        if delta == 'Left':
            state, state_updated = ('Sharpleft' if passed_int else 'forward'), True
            prev_turn = 'Left'
            did_turn = True  # robot turned
        elif delta == 'Right':
            state, state_updated = ('Sharpright' if passed_int else 'forward'), True
            prev_turn = 'Right'
            did_turn = True  # robot turned
        elif delta == 'None':
            # Robot went straight (no turn)
            did_turn = False

        # ---- Delayed saving of the direction after turning ----
        if did_turn == True :
            if passed_int and not delay_task_pending:
                # First time after passing intersection
                turn_event_time = time.ticks_ms()
                delay_task_pending = True
                passed_int = False  # reset, waiting for delay to finish
                print("Intersection passed. Will save direction after delay...")

            if delay_task_pending and time.ticks_diff(time.ticks_ms(), turn_event_time) >= delay_after_turn:
                
                save_direction()
                print('Saved new direction')
                delay_task_pending = False  # task completed

        if loop_count>9:
            loop_count, passed_int = 0, False
            
        # code to save the previous direction and only when the direction changes after delay get the new current direction by making the prev dir available
            

        if state=='forward':
            if R and not L:
                state, counter, state_updated = 'turn_right', 0, True
            elif L and not R:
                state, counter, state_updated = 'turn_left',  0, True
            elif L and C and R:
                state, counter, state_updated = 'turn_left',  0, True

        elif state=='turn_right':
            counter += 1
            if counter>=MAX_TURNS:
                state, counter, state_updated = 'forward', 0, True

        elif state=='turn_left':
            counter += 1
            if counter>=MAX_TURNS:
                state, counter, state_updated = 'forward', 0, True

      

        if state_updated:
            client.send((state+'\n').encode())
            print("Sent state:", state)

    sleep(0.04)


