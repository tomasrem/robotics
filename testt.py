import socket

HOST = '0.0.0.0'  # Listen on all interfaces (or use your IP)
PORT = 5678       # Use a different port than Webots

def handle_data(data):
    print(f"Received: {data}")
    # Do something with the data...

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.bind((HOST, PORT))
    s.listen(1)
    print(f"Listening on port {PORT}...")
    conn, addr = s.accept()
    with conn:
        print(f"Connected by {addr}")
        while True:
            data = conn.recv(1024)
            if not data:
                break
            handle_data(data.decode())

