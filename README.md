# projet_esp32cam

## Demo de base

Dans `demo` le fichier `demo.ino` correspond à la demo de base, elle fonctionne avec le serveur python `serveur_photo.py`.
C'est une demo, le serveur est très mal fait.
Les autres serveurs/clients servent à des tests.

## Demo Linux

Dans `demo-linux` le fichier `demo-linux.ino` correspond à la seconde demo. Celle ci utilise le serveur linux ci dessous, plus propre, en C++, et plus proche de l'integration finale.

## Serveur

 Serveur linux à utiliser avec les ESP. Ce serveur permet aussi la conversion de l'image en objet opencv Mat. On peut le retrouver dans le dossier `serveur` sous le nom `serveur.cpp`.
 Dans le même dossier, les protocoles sont expliqués dans les fichiers `protocole-esp.md` pour le côté client et `protocole-serveur.md` pour le côté serveur.

 > Nécessite l'installation d'OpenCV ([tuto](https://docs.opencv.org/4.x/d7/d9f/tutorial_linux_install.html))

## Demo Ethernet

Dans `demo-ethernet` le fichier `demo-ethernet.ino` correspond à la troisième demo. Celle ci utilise le même serveur que la demo precedente mais permet d'utilser une connection Ethernet avec l'ESP plutôt que le Wifi.

> /!\ Utilise la bibliothèque `EthernetSPI2.h` téléchargeable [ici](https://github.com/arhi/EthernetSPI2)

## Codes exemples

Dans `exemples` on trouve les fichiers exemples, trouvés dans les bibliotèques Arduino ou esp32 ou sur le web, que j'ai utilisés pour construire ma demo.
