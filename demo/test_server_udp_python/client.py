import socket, os

## Fichier

os.chdir('envoi')
nomFich = "loading_cat.jpg"
octets = os.path.getsize(nomFich)



## Setup socket

# IP du serveur et port
HOST = '172.20.10.3'
PORT = 61754

# Taille du buffer (laisser à 1024)
BUFFER = 1024

# Creation du socket
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)


## Envoi

with open(nomFich, 'rb') as f :
    for i in range (octets // BUFFER) :
        s.sendto(f.read(BUFFER), (HOST, PORT))
    if octets % BUFFER > 0 :
        s.sendto(f.read(), (HOST, PORT))

# send data to server
#s.sendto(bytearray(data, 'utf-8'), (HOST, PORT))

# Receive response from server
# data is a list with 2 elements
# first is data
# second is client address
#data = s.recvfrom(BUFFER)
#print('Server to Client: ' , data)

## Fermeture

s.close()