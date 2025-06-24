import matplotlib.pyplot as plt
from PIL import Image
import ast
import os
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





def main():
    # --- CONFIG ---
    image_file = "image.png"  # Make sure your map image is named this or adjust the name
    max_x = 16
    max_y = 12

    if not os.path.exists(image_file):
        print(f"[ERROR] Image file '{image_file}' not found.")
        return

    # Load the map image
    img = Image.open(image_file)

    # Ask user for path input
    path_input = input("Paste the path (e.g. [(16,10),(16,9),(16,8)]):\n").strip()
    try:
        path = ast.literal_eval(path_input)
    except Exception as e:
        print("[ERROR] Invalid path format:", e)
        return

    if not path or not isinstance(path, list) or not all(isinstance(p, tuple) for p in path):
        print("[ERROR] Invalid path structure.")
        return

    # Shift x and y by 0.5 to center on grid
    x_vals = [p[0] + 0.5 for p in path]
    y_vals = [p[1] + 0.5 for p in path]

    # Plot image and overlay path
    plt.figure(figsize=(7, 6))
    plt.imshow(img, extent=[0, max_x + 1, max_y + 1, 0])  # Keep image aligned
    plt.plot(x_vals, y_vals, marker='o', color='cyan', linewidth=2, markersize=6, label='Path')
    plt.title("Overlayed Path on Map")
    plt.xlim(0, max_x + 1)
    plt.ylim(max_y + 1, 0)
    plt.grid(False)
    plt.legend()
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()
