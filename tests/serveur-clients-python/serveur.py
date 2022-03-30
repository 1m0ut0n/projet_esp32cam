import socket, os


## Setup Socket

# Observer toutes les ips
HOST = '0.0.0.0'
# Port
PORT = 44444

# Taille du buffer (laisser à 1024)
BUFFER_SIZE = 1024

# Socket
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
# Bind du socket
s.bind((HOST, PORT))


## Reception

while True:
    # Receive BUFFER_SIZE bytes data
    # data is a list with 2 elements
    # first is data
    #second is client address
    data = s.recvfrom(BUFFER_SIZE)
    if data:
        #print received data
        print('Client to Server: ' , data)


        # Convert to upper case and send back to Client
        #s.sendto(bytearray('Received', 'utf-8'), data[1])


## Fermeture

s.close()