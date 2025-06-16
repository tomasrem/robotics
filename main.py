from machine import Pin, UART
from time import sleep
import itertools
from heapq import heappush, heappop

#--------------------------------------------------------------------------------------------------------

# DIJKSTRA ALGORITHM PART

class Graph:
    def _init_(self, adjacency_list):
        self.adjacency_list = adjacency_list


class Vertex:
    def _init_(self, value):
        self.value = value


class Edge:
    def _init_(self, distance, vertex):
        self.distance = distance
        self.vertex = vertex


def dijkstra(graph, start, end):
    previous = {v: None for v in graph.adjacency_list.keys()}
    visited = {v: False for v in graph.adjacency_list.keys()}
    distances = {v: float("inf") for v in graph.adjacency_list.keys()}
    distances[start] = 0
    queue = PriorityQueue()
    queue.add_task(0, start)
    path = []
    while queue:
        removed_distance, removed = queue.pop_task()
        visited[removed] = True

        if removed is end:
            while previous[removed]:
                path.append(removed.value)
                removed = previous[removed]
            path.append(start.value)
            print(f"shortest distance to {end.value}: ", distances[end])
            print(f"path to {end.value}: ", path[::-1])
            return path[::-1], distances[end]

        for edge in graph.adjacency_list[removed]:
            if visited[edge.vertex]:
                continue
            new_distance = removed_distance + edge.distance
            if new_distance < distances[edge.vertex]:
                distances[edge.vertex] = new_distance
                previous[edge.vertex] = removed
                queue.add_task(new_distance, edge.vertex)
    return


# slightly modified heapq implementation from https://docs.python.org/3/library/heapq.html
class PriorityQueue:
    def _init_(self):
        self.pq = []  # list of entries arranged in a heap
        self.entry_finder = {}  # mapping of tasks to entries
        self.counter = itertools.count()  # unique sequence count

    def _len_(self):
        return len(self.pq)

    def add_task(self, priority, task):
        'Add a new task or update the priority of an existing task'
        if task in self.entry_finder:
            self.update_priority(priority, task)
            return self
        count = next(self.counter)
        entry = [priority, count, task]
        self.entry_finder[task] = entry
        heappush(self.pq, entry)

    def update_priority(self, priority, task):
        'Update the priority of a task in place'
        entry = self.entry_finder[task]
        count = next(self.counter)
        entry[0], entry[1] = priority, count

    def pop_task(self):
        'Remove and return the lowest priority task. Raise KeyError if empty.'
        while self.pq:
            priority, count, task = heappop(self.pq)
            del self.entry_finder[task]
            return priority, task
        raise KeyError('pop from an empty priority queue')


# testing the algorithm
vertices = [Vertex("A"), Vertex("B"), Vertex("C"), Vertex("D"), Vertex("E"), Vertex("F"), Vertex("G"), Vertex("H"), Vertex("I"), Vertex("J"), Vertex("K"), Vertex("L"), Vertex("M"), Vertex("N"), Vertex("Ñ"), Vertex("O"), Vertex("P"), Vertex("Q"), Vertex("R")]
A, B, C, D, E, F, G, H, I, J, K, L, M, N, Ñ, O, P, Q, R = vertices

adj_list = {
    A: [Edge(0.1, B), Edge(0.25, G)],
    B: [Edge(0.1, A), Edge(0.1, C)],
    C: [Edge(0.1, B), Edge(0.1, D)],
    D: [Edge(0.1, C), Edge(0.2, E)],
    E: [Edge(0.2, D), Edge(0.5, F), Edge(0.16671, H)],
    F: [Edge(0.5, E), Edge(0.1667, I)],
    G: [Edge(0.25, A), Edge(0.501, J), Edge(0.5, L)],
    H: [Edge(0.16671, E), Edge(0.501, I), Edge(0.0833, J)],
    I: [Edge(0.1667, F), Edge(0.501, H),  Edge(0.0833, K)],
    J: [Edge(0.501, G), Edge(0.0833, H), Edge(0.501, K), Edge(0.0833, M)],
    K: [Edge(0.501, J), Edge(0.0833, I), Edge(0.25, R)],
    L: [Edge(0.0833, G), Edge(0.501, M), Edge(0.1667, N)],
    M: [Edge(0.0833, J), Edge(0.501, L), Edge(0.16671, Ñ)],
    N: [Edge(0.5, Ñ), Edge(0.1667, L)],
    Ñ: [Edge(0.5, N), Edge(0.16671, M), Edge(0.2, O)],
    O: [Edge(0.2, Ñ), Edge(0.1, P)],
    P: [Edge(0.1, O), Edge(0.1, Q)],
    Q: [Edge(0.1, P), Edge(0.1, R)],
    R: [Edge(0.1, Q), Edge(0.25, K)],
}

my_graph = Graph(adj_list)

#dijkstra(my_graph, start=A, end=N)

A_coordinate = [0, 0]
B_coordinate = [0, 0.1]
C_coordinate = [0, 0.2]
D_coordinate = [0, 0.3]
E_coordinate = [0, 0.5]
F_coordinate = [0, 1]
G_coordinate = [2.25, 0]
H_coordinate = [0.166, 0.5]
I_coordinate = [0.166, 1]
J_coordinate = [0.25, 1]
K_coordinate = [0.25, 1]
L_coordinate = [0.33, 0]
M_coordinate = [0.33, 0.5]
N_coordinate = [0.5, 0]
Ñ_coordinate = [0.5, 0]
O_coordinate = [0.5, 0.7]
P_coordinate = [0.5, 0.8]
Q_coordinate = [0.5, 0.9]
R_coordinate = [0.5, 1]


#--------------------------------------------------------------------------------------------------------
# UART COMMUNICATION PART

led_board = Pin(2, Pin.OUT)    
led_yellow = Pin(4, Pin.OUT)
led_blue = Pin(23, Pin.OUT)
led_green = Pin(22, Pin.OUT)
led_red = Pin(21, Pin.OUT)

button_left = Pin(16, Pin.IN, Pin.PULL_DOWN)
button_right = Pin(35, Pin.IN, Pin.PULL_DOWN)

robot_path_msg, _ = dijkstra(my_graph, start=A, end=P)
print (robot_path_msg)
nodes_to_travel = len(robot_path_msg)

robot_path_string = "".join(robot_path_msg)


path_in_coordinates = []

for i in range(nodes_to_travel):
    if robot_path_msg[i] == "A":
        path_in_coordinates.append(A_coordinate)
    elif robot_path_msg[i] == "B":
        path_in_coordinates.append(B_coordinate)
    elif robot_path_msg[i] == "C":
        path_in_coordinates.append(C_coordinate)
    elif robot_path_msg[i] == "D":
        path_in_coordinates.append(D_coordinate)
    elif robot_path_msg[i] == "E":
        path_in_coordinates.append(E_coordinate)
    elif robot_path_msg[i] == "F":
        path_in_coordinates.append(F_coordinate)
    elif robot_path_msg[i] == "G":
        path_in_coordinates.append(G_coordinate)
    elif robot_path_msg[i] == "H":
        path_in_coordinates.append(H_coordinate)
    elif robot_path_msg[i] == "I":
        path_in_coordinates.append(I_coordinate)
    elif robot_path_msg[i] == "J":
        path_in_coordinates.append(J_coordinate)
    elif robot_path_msg[i] == "K":
        path_in_coordinates.append(K_coordinate)
    elif robot_path_msg[i] == "L":
        path_in_coordinates.append(L_coordinate)
    elif robot_path_msg[i] == "M":
        path_in_coordinates.append(M_coordinate)
    elif robot_path_msg[i] == "N":
        path_in_coordinates.append(N_coordinate)
    elif robot_path_msg[i] == "Ñ":
        path_in_coordinates.append(Ñ_coordinate)
    elif robot_path_msg[i] == "O":
        path_in_coordinates.append(O_coordinate)
    elif robot_path_msg[i] == "P":
        path_in_coordinates.append(P_coordinate)
    elif robot_path_msg[i] == "Q":
        path_in_coordinates.append(Q_coordinate)
    elif robot_path_msg[i] == "R":
        path_in_coordinates.append(R_coordinate)

print(path_in_coordinates) 


print("Click the button on the ESP32 to continue. Then, close Thonny and run the Webots simulation.")

while button_left() == False:
    sleep(0.25)
    led_board.value(not led_board())

uart = UART(1, 115200, tx=1, rx=3)

# Variables to implement the line-following state machine
current_state = 'IDLE'
state_updated = True

leido = False

while True:
    if uart.any():
        msg = uart.readline()
        if msg:
            msg_str = msg.decode('utf-8').strip()    
                
            if msg_str == "IDLE":
                led_blue.off()
                led_green.off()
                led_red.off()
                led_yellow.on()
                if leido == False:
                    for i in range(10):
                        uart.write(robot_path_string +'\n')
                
                        
            if msg_str == "forward":
                led_blue.on()
                led_green.off()
                led_red.off()
                led_yellow.off() 
                
            elif msg_str == "left":
                led_blue.off()
                led_green.on()
                led_red.off()
                led_yellow.off()
                
            elif msg_str == "right":
                lef_blue.off()
                led_green.off()
                led_red.on()
                led_yellow.off()

    if state_updated == True:
        uart.write(robot_path_string + '\n')
        estado = current_state
        state_updated = False

    sleep(0.02)