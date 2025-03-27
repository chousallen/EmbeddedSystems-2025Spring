import socket
import select
import sys
import time

def receive_data(data):
    data = data.split(',')
    if len(data) == 3:
        x = float(data[0])
        y = float(data[1])
        z = float(data[2])

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
                            s.sendall("Message received".encode('utf-8'))
                        else:
                            inputs.remove(s)
                            s.close()
                    except BlockingIOError:
                        pass
    finally:
        server_socket.close()

if __name__ == "__main__":
    run_tcp_server()
