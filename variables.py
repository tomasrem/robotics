# orientation of the robot from degrees 
threshold = 30
val = raw_phi
print(raw_phi)
# Determine direction
if abs(val - 90) <= threshold:
    direction = 'U'
elif abs(val - 180) <= threshold:
    direction = 'L'
elif val <= 40 or val > 320 :
    direction = 'R'
elif abs(val - 270 ) <= threshold:
    direction = 'D'
else:
    direction = None
    
delta = 'None'   
if direction == 'U':
    if (tx - gx) == -1:
        #turn left
        delta = 'Left'
    if (tx - gx) == 1:
        #turn right
        delta = 'Right'
        
if direction == 'D':
    if (tx - gx) == 1:
        #turn left
        delta = 'Left'
    if (tx - gx) == -1:
        #turn right
        delta = 'Right'
        
if direction == 'L':
    if (ty - gy) == -1:
        #turn left
        delta = 'Left'
    if (ty - gy) == 1:
        #turn right
        delta = 'Right' 

if direction == 'R':
    if (ty - gy) == 1:
        #turn left
        delta = 'Left'
    if (ty - gy) == -1:
        #turn right
        delta = 'Right' 


points = {
    # Point name , grid location (X , Y ) , real position x,y
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

# Access examples
# print(points['D'])    # ((16, 10), (0, 0))
# print(points['J'][0]) # (8, 6)
# print(points['J'][1]) # (-0.5, 0.24)


# --- Grid and Cost Definitions --- X in grid in points varaible represents the position of number in the list and Y is the number of list eg row so to acces it its grig[0][1] is x 0 and y 1
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
     [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
     [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
     [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]
    ]



SSID = 'Ziggo1541315'
PASSWORD = 'pchcestzy3gfnvdV'

STATIC_IP = '192.168.178.73'
SUBNET    = '255.255.255.0'
GATEWAY   = '192.168.178.1'
DNS       = '8.8.8.8'

def connect_wifi_static(ssid, password, ip, subnet, gateway, dns, timeout=15):
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    # Configure static interface
    wlan.ifconfig((ip, subnet, gateway, dns))
    print('Static IP configured:', wlan.ifconfig())
    # Now connect
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

# Bring up Wi-Fi with static IP
connect_wifi_static(SSID, PASSWORD,
                    STATIC_IP, SUBNET,
                    GATEWAY, DNS)



# --- Wi-Fi / TCP setup (do not remove) ---
addr = socket.getaddrinfo('0.0.0.0', 1234)[0][-1]
server = socket.socket()
server.bind(addr)
server.listen(1)
print('Listening on', addr)

client, _ = server.accept()
print('Client connected!')