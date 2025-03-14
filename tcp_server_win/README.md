# TCP Server for Windows

This project contains a simple TCP server implemented in C++ for Windows.

## Prerequisites

- MinGW-w64 (for g++)
- Windows operating system

## Compilation

To compile the TCP server, open a terminal and navigate to the project directory. Then run the following command:

```sh
g++ .\tcp_server.cpp -lws2_32 -o tcp_server
```

This command compiles the `tcp_server.cpp` file and links it with the Winsock library.

## Running the Server

After compiling, you can run the server with the following command:

```sh
.\tcp_server
```

The server will start and listen for incoming connections on port 8080. It will echo any received data back to the client.

## Notes

- Make sure that port 8080 is not blocked by your firewall.
- You can change the port by modifying the `DEFAULT_PORT` macro in the `tcp_server.cpp` file.
