import heapq
from time import sleep
import socket

# --- Wi-Fi / TCP setup (do not remove) ---
addr = socket.getaddrinfo('0.0.0.0', 1234)[0][-1]
server = socket.socket()
server.bind(addr)
server.listen(1)
print('Listening on', addr)

client, _ = server.accept()
print('Client connected!')

# --- Tunable cell sizes (meters) and origin in grid coords ---
cell_sizex  = 0.058823529
cell_sizey  = 0.061538461
origin_col  = 16
origin_row  = 10

def world_to_grid(x, y):
    """
    Convert real-world (x,y) → grid (col,row).
    Uses (origin_col,origin_row) as the grid cell for real (0,0).
    """
    col_f = origin_col + (x / cell_sizex)
    row_f = origin_row - (y / cell_sizey)
    return int(round(col_f)), int(round(row_f))

# --- Named waypoints: (grid-col,grid-row), (real-x,real-y) ---
points = {
    'A': ((10, 10), (-0.31, 0)),
    'B': ((12, 10), (-0.205, 0)),
    'C': ((14, 10), (-0.11, 0)),
    'D': ((16, 10), (0, 0)),
    'E': ((16, 6), (0, 0.24)),
    'F': ((16, 4), (0, 0.35)),
    'G': ((16, 2), (0, 0.5)),
    'H': ((8, 2), (-0.5, 0.5)),
    'I': ((8, 4), (-0.5, 0.35)),
    'J': ((8, 6), (-0.5, 0.24)),
    'K': ((8, 8), (-0.5, 0.145)),
    'L': ((8, 10), (-0.5, 0)),
    'M': ((0, 10), (-1, 0)),
    'N': ((0, 8), (-1, 0.145)),
    'O': ((0, 6), (-1, 0.24)),
    'P': ((0, 2), (-1, 0.5)),
    'R': ((2, 2), (-0.895, 0.5)),
    'S': ((4, 2), (-0.795, 0.5)),
    'T': ((6, 2), (-0.69, 0.5)),
}

# --- Reverse lookup: (col,row) → (label, real_x, real_y) ---
grid_to_waypoint = {
    points[label][0]: (label, points[label][1][0], points[label][1][1])
    for label in points
}

# --- Occupancy grid (0=free, 1=wall) ---
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

# --- Uniform move cost ---
costs = [[1]*17 for _ in range(13)]
# example of a higher cost cell:
costs[10][15] = 10

# --- Dijkstra with tie-breaker for preferred direction ---
def dijkstra(grid, costs, start, goal):
    cols = len(grid[0])
    rows = len(grid)
    visited   = set()
    distances = {start: 0}
    parents   = {start: None}
    frontier  = [(0, 0, start)]   # (dist, priority, node)

    # Preferred move order: up, left, down, right
    moves = [ (0, -1), (-1, 0), (0, 1), (1, 0) ]

    while frontier:
        current_distance, _, current = heapq.heappop(frontier)
        if current in visited:
            continue
        visited.add(current)
        if current == goal:
            break

        for order, (dx, dy) in enumerate(moves):
            neigh = (current[0] + dx, current[1] + dy)
            c, r = neigh
            if not (0 <= c < cols and 0 <= r < rows):
                continue
            if grid[r][c] == 1:
                continue

            new_dist = current_distance + costs[r][c]
            if new_dist < distances.get(neigh, float('inf')):
                distances[neigh] = new_dist
                parents[neigh]   = current
                heapq.heappush(frontier, (new_dist, order, neigh))

    # Reconstruct path
    path, node = [], goal
    while node is not None:
        path.append(node)
        node = parents.get(node)
    return list(reversed(path))

def label_to_node(label):
    return points[label][0]

# --- Plan once ---
start_label = 'D'
goal_label  = 'P'
start_node  = label_to_node(start_label)
goal_node   = label_to_node(goal_label)

full_path = dijkstra(grid, costs, start_node, goal_node)
# drop the start node so we don't revisit it
path = full_path[1:] if full_path and full_path[0] == start_node else full_path

print("Planned path (col,row):")
for step in path:
    print(step)

# --- Loop state init ---
buffer               = ""
loop_count           = 0
passed_intersection  = False
in_intersection      = False
current_state        = 'forward'
counter              = 0
COUNTER_MAX          = 5
COUNTER_STOP         = 20
calibrate_x_offset   = 0.0
calibrate_y_offset   = 0.0
calibrate_phi_offset = 0.0
idx                  = 0

while True:
    loop_count += 1
    state_updated = False

    data = client.recv(64).decode()
    buffer += data

    while '\n' in buffer:
        line, buffer = buffer.split('\n', 1)
        parts = line.split(',')
        if len(parts) < 4:
            continue

        bits = parts[0].strip()
        try:
            raw_x   = int(parts[1]) / 1000.0
            raw_y   = int(parts[2]) / 1000.0
            raw_phi = int(parts[3]) / 1000.0
        except ValueError:
            print("Invalid data:", parts)
            continue

        L = (bits[0] == '1')
        C = (bits[1] == '1')
        R = (bits[2] == '1')

        # --- Calibrate on every intersection visit ---
        now_int = not (L or C or R)
        if now_int and not in_intersection:
            gx_raw, gy_raw = world_to_grid(raw_x, raw_y)
            key = (gx_raw, gy_raw)
            if key in grid_to_waypoint:
                lbl, rx, ry = grid_to_waypoint[key]
                calibrate_x_offset   = raw_x   - rx
                calibrate_y_offset   = raw_y   - ry
                calibrate_phi_offset = raw_phi
                print(f"Calibrated at {lbl}: x_off={calibrate_x_offset:.3f}, "
                      f"y_off={calibrate_y_offset:.3f}, φ_off={calibrate_phi_offset:.3f}")
            in_intersection = True
        if not now_int and in_intersection:
            in_intersection = False

        # --- Apply calibration ---
        x   = raw_x   - calibrate_x_offset
        y   = raw_y   - calibrate_y_offset
        phi = raw_phi - calibrate_phi_offset

        # --- What cell are we in now? ---
        gx, gy = world_to_grid(x, y)

        # --- What cell are we heading to? ---
        if idx < len(path):
            tx, ty = path[idx]
        else:
            tx, ty = goal_node

        print(f"Grid: ({gx},{gy}) → target: ({tx},{ty})")

        # --- Reached waypoint? ---
        if gx == tx and gy == ty:
            if (tx, ty) in grid_to_waypoint:
                lbl, rx, ry = grid_to_waypoint[(tx, ty)]
                print(f"✓ Reached waypoint {lbl}: true pos=({rx:.3f},{ry:.3f})")
            else:
                print(f"✓ Reached grid cell ({tx},{ty})")
            idx += 1
            if idx >= len(path):
                current_state = 'stop'
            state_updated = True

        # --- Sharp‐turn & line follow FSM (unchanged) ---
        delta = tx - gx
        if not (L or C or R):
            passed_intersection = True

        if delta == -1:
            current_state = 'Sharpleft' if passed_intersection else 'forward'
            state_updated = True
        elif delta == 1:
            current_state = 'Sharpright' if passed_intersection else 'forward'
            state_updated = True

        if loop_count > 9:
            loop_count = 0
            passed_intersection = False

        if current_state == 'forward':
            if R and not L:
                current_state = 'turn_right'
                counter = 0
                state_updated = True
            elif L and not R:
                current_state = 'turn_left'
                counter = 0
                state_updated = True
            elif L and C and R:
                current_state = 'turn_left'
                counter = 0
                state_updated = True

        elif current_state == 'turn_right':
            counter += 1
            if counter >= COUNTER_MAX:
                current_state = 'forward'
                counter = 0
                state_updated = True

        elif current_state == 'turn_left':
            counter += 1
            if counter >= COUNTER_MAX:
                current_state = 'forward'
                counter = 0
                state_updated = True

        elif current_state == 'stop':
            # assume led_board is defined elsewhere
            led_board.on()
            counter += 1
            if counter >= COUNTER_STOP:
                current_state = 'forward'
                counter = 0
                led_board.off()
                state_updated = True

        if state_updated:
            client.send((current_state + '\n').encode())
            print("Sent state:", current_state)

    sleep(0.05)

