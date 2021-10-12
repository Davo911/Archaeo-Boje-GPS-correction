import socket
import sys
import time

#Function to send a string to UDP-Address 192.168.2.2 on port 27000
def send_string(string):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.sendto(string, ("192.168.2.2", 27000))


# Create a TCP/IP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Bind the socket to the port
server_address = ('127.0.0.1', 12001)
print('starting up on {} port {}'.format(*server_address))
sock.bind(server_address)

# Listen for incoming connections
sock.listen(1)

while True:
    # Wait for a connection
    print('waiting for a connection')
    connection, client_address = sock.accept()
    try:
        print('connection from', client_address)

        # Receive the data in small chunks and retransmit it
        while True:
            data = connection.recv(2048)
            print('received {!r}'.format(data))
            if data:
                print('sending data back to UDP')
                send_string(data)
            else:
                print('no data from', client_address)
                break

    finally:
        # Clean up the connection
        connection.close()