# projet_esp32cam


Projet réalisé lors de mon stage en entreprise de l'année L1 (8 semaines).

Le but était d'interfacer un esp32 équipé d'une caméra à un serveur afin d'envoyer des photos de produits en sortie de production pour que le serveur puisse les analyser avec une IA et en détecter les problèmes. Le projet s'est fait en plusieurs étapes à messure des difficulté rencontrée ou des améliorations apportées. Les différentes `demo-xxxx` correspondent à chacunes des étapes de réalisation. D'autres choses ont été necessaires pour le projet comme la réalisation des serveurs de réception, l'écriture d'une bibliothèque Ethernet custom et différents programmes de tests.




## Demos


### Demo de base

La première étape fut de savoir si se qu'on voulait faire été possible. Le but été donc d'aller au plus simple, d'où le serveur python et le WiFi.

Dans `demo` le fichier `demo.ino` correspond donc à la demo de base, elle fonctionne avec le serveur python et communique en WiFi.
C'est une sorte de demo prototype, le serveur est donc très mal fait.
Des autres serveurs/clients python de test sont présents dans `tests`.


### Demo Linux

La prochaine étape était donc de créer un serveur et un protocole de communication plus propre.

Dans `demo-linux` le fichier `demo-linux.ino` correspond donc à la seconde demo.
Celle ci utilise le premier serveur linux situé dans `serveur`, plus propre, en C++, et plus proche de l'integration finale.


### Demo Ethernet

Ensuite, pour palier à l'instabilité du WiFi dans un environement industriel, on préférait utiliser une connexion Ethernet plutôt que le WiFi. La troisième étape consistait donc à relier un shield Ethernet et à adapter le code à celui-ci.

Dans `demo-ethernet` le fichier `demo-ethernet.ino` correspond donc à la troisième demo. Celle ci utilise le même serveur que la demo précédente mais permet en plus d'utiliser une connection Ethernet avec l'ESP plutôt que le Wifi.

> /!\ Utilise la bibliothèque `EthernetSPI2.h` téléchargeable [ici](https://github.com/arhi/EthernetSPI2)


### Demo Opti

Dans l'optique d'ameliorer les temps de transmission, qui devaient s'approcher du temps réel, on a preferé réécrire les bibliothèques permettant la communication afin de ne l'adapter qu'a nos besoins.
La bibliothèque `EthernetSPI2.h` était en effet adaptée plusieurs shield Ethernet et protocoles différents, necessitant un temps d'execution supplementaire (et de la mémoire).

Dans `demo-opti` le fichier `demo-opti.ino` correspond donc à la quatrième demo.
Elle utilise les bibliothèques customs `EthernetUDP.h` et `w5500HSPI.h` qui remplacent donc l'ancienne bibliothèque Ethernet.

> Il faut copier les bibliothèques, soit les fichiers `w5500HSPI.h`, `w5500HSPI.cpp`, `EthernetUDP.h` et `EthernetUDP.cpp` dans le dossier `demo-opti` pour que la compilation puisse se faire


### Demo Protocole

Toujours dans le but de reduire les temps d'envoi, on décide de modifier le protocole de transmission de l'image pour qu'il necessite moins d'aller/retour serveur-client et qu'il soit plus intelligent.
Il a donc aussi nécéssité un nouveau serveur.

Dans `demo-protocole` le fichier `demo-protocole.ino` correspond donc à la cinquième demo.
Elle communique avec le second serveur linux situé dans `serveurs` et utilise un nouveau protocole de communication.
Elle utilise aussi les mêmes bibliothèques que la demo précédente.

> Il faut copier les bibliothèques, soit les fichiers `w5500HSPI.h`, `w5500HSPI.cpp`, `EthernetUDP.h` et `EthernetUDP.cpp` dans le dossier `demo-protocole` pour que la compilation puisse se faire




## Serveurs


### Serveur Python

Serveur linux à utiliser avec la première demo (prototype).
Il est assez mal fait, gère peu de cas critique et est peu optimisé mais ça n'est qu'un prototype.
On peut le retrouver dans le dossier `serveur-python` sous le nom `serveur_photo.py`.


### Serveur Linux

Premier serveur linux à utiliser avec les demo 2, 3 et 4 (ancien protocole).
Ce serveur permet aussi la conversion de l'image en objet opencv Mat.
On peut le retrouver dans le dossier `serveur-linux` sous le nom `serveur.cpp`.
Dans le même dossier, les protocoles sont expliqués dans les fichiers `protocole-esp.md` pour le côté client et `protocole-serveur.md` pour le côté serveur.

> Nécessite l'installation d'OpenCV ([tuto](https://docs.opencv.org/4.x/d7/d9f/tutorial_linux_install.html))

> Necessite la creation d'un dossier `reception` pour que l'image puisse s'enregistrer


### Serveur Protocole

Second serveur linux à utiliser avec la demo 5 (nouveau protocole). Ce serveur permet aussi la conversion de l'image en objet opencv Mat. On peut le retrouver dans le dossier `serveur-protocole` sous le nom `serveur.cpp`.
Dans le même dossier, les protocoles sont expliqués dans les fichiers `protocole-esp.md` pour le côté client et `protocole-serveur.md` pour le côté serveur.

> Nécessite l'installation d'OpenCV ([tuto](https://docs.opencv.org/4.x/d7/d9f/tutorial_linux_install.html))

> Necessite la creation d'un dossier `reception` pour que l'image puisse s'enregistrer



## Bibliothèque

Dans `library` on trouve les bibliothèques customs necessaire aux demos 4 et 5.
La bibliothèque `w5500HSPI.h` sert à communiquer avec la w5500 du shield Ethernet et la bibliothèque `EthernetUDP.h` donne les fonctions liées à l'Ethernet et la classe UDP.
Ces bibliothèques sont principalements adaptés à notre cas de figure précis et sont assez difficilement adaptable.
Par contre, elle sont plus rapides et économes que la tradituionelle bibliothèque `Ethernet.h`.

> Les bibliothèques customs avaient été écrites pour intégrer un mode de configuration en DHCP qui n'est pas encore présent. La demo ne fonctionne donc pour l'instant qu'en IP fixe.



## Tests


### Serveur/Client Python

Serveur et Client de test en Python pour la toute première demo.


### Tests Demo Opti

Programmes utilisées pour tester les bibliothèques custom réaliséees lors de la quatrième demo. `serveur-envoi.c` est un serveur en C++ qui permet de rentrer une phrase et de l'envoyer en UDP à une IP définie.
`serveur-reception.c` est un serveur en C++ qui affiche tout les messages recu en UDP sur un port défini.
`test-demo-opti.ino` est le programme de test pour les bibliothèques customs, il affichent les messages reçus en UDP sur un port défini et permet d'envoyer des phrases à une adresse IP définie.


### w5500 HSPI

Test de la communication SPI entre un shield Ethernet équipé d'une w5500 et l'esp32
