import matplotlib.pyplot as plt
import matplotlib.animation as animation
from queue import Queue
import random
import time
import socket
import select

# Shared queues for real-time data
X = Queue()
Y = Queue()
Z = Queue()
T = Queue()

# Initialize the figure and axis
fig, ax = plt.subplots()
line_x, = ax.plot([], [], lw=2, label='X')  # Line for X
line_y, = ax.plot([], [], lw=2, label='Y')  # Line for Y
line_z, = ax.plot([], [], lw=2, label='Z')  # Line for Z
# ax.set_xlim(0, 100)  # Set x-axis range
ax.set_ylim(-2000, 2000)  # Set y-axis range
ax.set_title("Real-Time Line Chart")
ax.set_xlabel("Time")
ax.set_ylabel("Value")
ax.legend()  # Add legend for the lines

# Initialize data storage
x_data = []
y_data = []
z_data = []
t_data = []

def receive_data(data):
    data = data.split('\n')[0]
    data = data.split(',')
    if len(data) == 4:
        T.put(float(data[0]))
        X.put(float(data[1]))
        Y.put(float(data[2]))
        Z.put(float(data[3]))

def run_tcp_server(host='0.0.0.0', port=8080):
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind((host, port))
    server_socket.listen(5)
    server_socket.setblocking(False)  # Set the server socket to non-blocking mode
    print(f"Server listening on {host}:{port}")

    inputs = [server_socket]
    try:
        while True:
            readable, _, _ = select.select(inputs, [], [], 1)  # Use select for non-blocking I/O
            for s in readable:
                if s is server_socket:
                    try:
                        client_socket, client_address = server_socket.accept()
                        print(f"Connection established with {client_address}")
                        inputs.append(client_socket)
                    except BlockingIOError:
                        pass
                else:
                    try:
                        data = s.recv(1024).decode('utf-8')
                        if data:
                            print(f"Received: {data}")
                            receive_data(data)
                            s.sendall("Message received".encode('utf-8'))
                        else:
                            inputs.remove(s)
                            s.close()
                    except BlockingIOError:
                        pass
    finally:
        server_socket.close()

def init():
    """Initialize the line chart."""
    line_x.set_data([], [])
    line_y.set_data([], [])
    line_z.set_data([], [])
    return line_x, line_y, line_z

def update(frame):
    """Update the line chart with new data."""
    if not T.empty() and not X.empty() and not Y.empty() and not Z.empty():
        t_data.append(T.get())
        x_data.append(X.get())
        y_data.append(Y.get())
        z_data.append(Z.get())
        if len(t_data) > 100:  # Keep the last 100 points
            t_data.pop(0)
            x_data.pop(0)
            y_data.pop(0)
            z_data.pop(0)
        line_x.set_data(t_data, x_data)
        line_y.set_data(t_data, y_data)
        line_z.set_data(t_data, z_data)
        ax.set_xlim(min(t_data), max(t_data))  # Dynamically update x-axis range
    return line_x, line_y, line_z

# Start the animation
ani = animation.FuncAnimation(fig, update, init_func=init, blit=True, interval=100)

if __name__ == "__main__":
    import threading
    # Start the data simulation in a separate thread
    threading.Thread(target=run_tcp_server, daemon=True).start()
    # Show the plot
    plt.show()
