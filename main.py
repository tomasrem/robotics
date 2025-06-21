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
grid_to_waypoint = {
    loc: (lbl, tx, ty)
    for lbl, (loc, (tx, ty)) in points.items()
}

# --- Dijkstra pathfinding ---
costs = [[1]*COLS for _ in range(ROWS)]
costs[10][15] = 10

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
start_label, goal_label = 'D', 'P'
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
last_gx, last_gy = start_node

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
        if now_int and not in_int:
            gx_raw, gy_raw = world_to_grid(raw_x, raw_y)
            key = (gx_raw, gy_raw)
            if key in grid_to_waypoint:
                lbl, tx_true, ty_true = grid_to_waypoint[key]
                cal_x_off   = raw_x   - tx_true
                cal_y_off   = raw_y   - ty_true
                cal_phi_off = raw_phi
                # PRINT waypoint arrival, offsets, and true position
                print(f"✓ Reached waypoint {lbl}")
                print(f"  → offsets: x_off={cal_x_off:.3f}, y_off={cal_y_off:.3f}, φ_off={cal_phi_off:.3f}")
                print(f"  → calibrated to true pos: ({tx_true:.3f},{ty_true:.3f})")
            in_int = True
        if not now_int and in_int:
            in_int = False

        # --- Apply calibration to pose ---
        x   = raw_x   - cal_x_off
        y   = raw_y   - cal_y_off
        phi = raw_phi - cal_phi_off

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
            gx, gy = gx_raw, gy_raw

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
        delta = tx - gx
        if not (L or C or R):
            passed_int = True

        if delta == -1:
            state, state_updated = ('Sharpleft' if passed_int else 'forward'), True
        elif delta ==  1:
            state, state_updated = ('Sharpright' if passed_int else 'forward'), True

        if loop_count>9:
            loop_count, passed_int = 0, False

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

        elif state=='stop':
            led_board.on()  # assume defined elsewhere
            counter += 1
            if counter>=STOP_PERIOD:
                state, counter, state_updated = 'forward', 0, True
                led_board.off()

        if state_updated:
            client.send((state+'\n').encode())
            print("Sent state:", state)

    sleep(0.05)

