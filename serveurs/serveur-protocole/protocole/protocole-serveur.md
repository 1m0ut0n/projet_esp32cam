# [Côté serveur] Nouveau protocole d'envoi d'une image depuis l'esp32 (client) vers le serveur

Le serveur communique selon le protocole UDP, il reçoit donc des paquets de la part de clients, ici des ESP, qui lui enverront des données et pourront en demander en retour.
Toutes les communications Serveur-Client se passeront comme suit.

## Type de client

**Octet n°1 :** (*Entier non-signé*) Le premier octet de chaque trame donnera le type de client auquel le serveur s'adresse.
Les possibilités de clients sont :
- 0) Spécial
- 1) esp32-cam pour la photographie des câbles

## 1) Photographie des câble avec l'esp32-cam

On se situe ici dans le cas d'une réponse aux ESP qui serviront à la photographie des câbles.

**Octet n°2 :** (*Entier non-signé*) Identifiant unique de l'ESP  à qui il s'adresse parmi les autres ESP du même type.

**Octet n°3 :** (*Entier non-signé*) A quel type de communictaion le serveur donne sa réponse. Il y a plusieurs type de communication possibles :
- 0 : Confirmation de bonne reception
- 1 : Demande d'envoi d'une image et de son analyse par l'IA
- 2 : Envoi d'un paquet de données
- 3 : Indication de fin de l'envoi

###  1 : Demande d'envoi d'une image et de son analyse par l'IA

Ici, le serveur repond à la demande d'envoi d'une image par l'ESP

**Octet n°4 :** (*Entier non-signé*) Reponse du serveur. Pour un OK, on mettra 0, dans le cas contraire on mettra autre chose (pourrait être un code d'erreur).

**Octet n°5 :** (*Entier non-signé*) Redonne le nombre de paquets necessaire à l'envoi de l'image (Vérification).

**Octet n°6-9 :** (*Entier non-signé de 4 octets*) Redonne la taille maximum de chaque paquet (Vérification).

> L'ESP peut alors savoir si il peut engager l'envoi ou non. Si l'ESP n'as pas de réponse (TIMEOUT), le paquet sera renvoyé et si la réponse est négative la demande sera refaite un peu plus tard...

### 2 : Envoi d'un paquet de données

Le serveur n'as pas besoin de donner de reponse lors de la reception

### 3 : Indication de fin de l'envoi

Le serveur verifie alors si il a bien recu tout les paquets et si ce n'est pas le cas

**Octet n°4 :** (*Entier non-signé*) Nombre de paquets non reçu, on mettra 0 si tout les paquets on bien été reçus.

**Octet n°5-x :** (*Suite d'entiers non-signés*) Liste des identifiants des paquets non reçus (chaque identifiant est codé sur 1 octet).

> L'ESP vérifie alors la bonne réception de tout les paquets par le serveur grâce à cette réponse. Dans le cas ou le serveur n'aurait pas tout reçu, l'ESP renvoie alors uniquement les paquets non reçu puis renvoie une indication de fin. Si l'ESP n'as aucune de réponse (TIMEOUT), le paquet sera renvoyé.
