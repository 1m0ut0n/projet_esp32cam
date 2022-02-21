import socket
import os

## Mode serveur

# Reponse du serveur à une requette d'envoi de paquet
reponseRequete = True
reponseRequetePositive = True


## Fichier

os.chdir('reception')
nomImg = 'cam.jpg'


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

upload = False
run = True


def message(data) :
    # Decodage et affichage des message reçus
    msg = data[0].decode('utf-8')
    print('Reception :')
    print('   Message : '+ msg)
    print('   Emmeteur : ' + str(data[1]))
    return msg


def requette(msg, emmeteur) :
    # Reception d'une requette
    if msg[:19] == 'file upload request' :
        # Infos
        id_esp = msg[26:30]
        packet = msg[41:45]

        print("   -> Detection d'une requete d'envoi de " + str(int(packet)) + " paquets de la part de l'ESP n°" + str(int(id_esp)))

        # Reponse à la requete
        if reponseRequete :

            if reponseRequetePositive :
                # Reponse positive
                print("   <- Envoi d'une réponse positive")
                s.sendto(bytearray('file request accepted, id : ' + id_esp + ', packet : ' + packet, 'utf-8'), emmeteur)
                return True, int(id_esp), int(packet)

            else :
                # Reponse negative
                print("   <- Envoi d'une réponse negative")
                s.sendto(bytearray('file request refused, id : ' + id_esp + ', packet : ' + packet, 'utf-8'), emmeteur)
                return False, int(id_esp), int(packet)

        else :
            # Pas de reponse
            print("   <- Pas d'envoi de réponse")
            return False, int(id_esp), int(packet)


def uploadMode(esp, packet) :
            print('Reception de données :')

            # Suppression de l'ancien fichier s'il existe
            if os.path.exists(nomImg):
                os.remove(nomImg)

            # Ouverture d'un fichier image
            with open(nomImg, 'wb') as img :
                while True :
                    #Reception des paquets
                    data = s.recvfrom(BUFFER_SIZE)
                    if data :
                        msg = data[0][:43].decode('utf-8')
                        print('   Reception paquet')
                        print('      Message : '+ msg)
                        print('      Emmeteur : ' + str(data[1]))

                        # Dans le cas d'un paquet d'upload
                        if msg[:12] == 'file upload,' :
                            # Infos
                            id_esp = msg[18:22]
                            num_packet = msg[30:34]

                            if int(id_esp) == esp :

                                print("      -> Reception du paquet " + str(int(num_packet)+1) + "/" + str(int(packet)))
                                # Ecriture des données
                                img.write(data[0][43:])
                                print('         Ecriture')

                                #Envoi de la confirmation de reception
                                s.sendto(bytearray('ACK, id : ' + id_esp + ', idP : ' + num_packet, 'utf-8'), data[1])

                                #Fin de l'upload
                                if int(num_packet) >= int(packet) -1 :
                                    print("      Dernier paquet")
                                    return True



while run:
    # Reception des paquets
    data = s.recvfrom(BUFFER_SIZE)

    if data:

        # Decodage et affichage des message reçus
        emmeteur = data[1]
        msg = message(data)

        upload, esp, packet = requette(msg, emmeteur)

        if upload : # Si on est en mode upload

            if uploadMode(esp, packet) : #On recoit les paquets et si on a réussi
                print('Image reçue')
            else :
                print('Problème avec la reception')



## Fermeture

s.close()