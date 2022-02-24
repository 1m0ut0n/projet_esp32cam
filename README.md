# projet_esp32cam

## Demo de base

Dans `demo` le fichier `demo.ino` correspond à la demo de base, elle fonctionne avec le serveur python `serveur_photo.py`.
C'est une demo, le serveur est très mal fait.
Les autres serveurs/clients servent à des tests.
Dans `exemples` on trouve les fichiers exemples, trouvés dans les bibliotèques Arduino ou esp32 ou sur le web, que j'ai utilisés pour construire ma demo.

## Demo Linux

Dans `demo-linux` le fichier `demo-linux.ino` correspond à la seconde demo. Celle ci utilise un serveur sous linux, plus propre, en c, et plus proche de l'integration finale. Ce serveur permet aussi la conversion de l'image en objet opencv Mat. On peut le retrouver dans le dossier `serveur` sous le nom `serveur.c`.
Dans le même dossier, les protocoles sont expliqués dans les fichiers `protocole-esp.md` pour le côté client et `protocole-serveur.md` pour le côté serveur.
