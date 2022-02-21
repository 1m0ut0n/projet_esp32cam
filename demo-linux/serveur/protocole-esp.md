# Protocole de communication Client->Serveur

Le serveur communique selon le protocole UDP, il reçoit donc des paquets de la part de clients, ici des ESP, qui lui enverront des données et pourront en demander en retour.
Toutes les communications Client-Serveur se passeront comme suit.

## Type de client

**Octet n°1 :** (*Entier non-signé*) Le premier octet de chaque trame dira au serveur à quel type de client il a affaire, ce qui permettra de savoir quoi faire.
Les possibilités de clients sont :
- 0 : Spécial
- 1 : esp32-cam pour la photographie des câbles

## 1 : Photographie des câble avec l'esp32-cam

On se situe ici dans le cas des ESP qui serviront à la photographie des câbles.

**Octet n°2 :** (*Entier non-signé*) Identifiant unique de l'ESP parmi les autres ESP du même type.

**Octet n°3 :** (*Entier non-signé*) Type de communications engagée. Il y a plusieurs type de communication possibles.
- 0 : Confirmation de bonne reception
- 1 : Demande d'envoi d'une image et de son analyse par l'IA
- 2 : Envoi d'un paquet de données
- 3 : ...

###  1 : Demande d'envoi d'une image et de son analyse par l'IA

Ici, l'ESP demande au serveur s'il peut lui envoyer une image pour analyse avec l'IA.

**Octet n°4 :** (*Entier non-signé*) Nombre de paquets necessaire à l'envoi de l'image.

**Octet n°5-8 :** (*Entier non-signé de 4 octets*) Taille maximum de chaque paquet.

**Octet restants :** (*Entier non-signé de n octets*) Taille du fichier image (pour confirmation de bonne reception).

> Après cette demande le serveur doit repondre si il accepte ou non. Voir `protocole-serveur.md` pour ça.

### 2 : Envoi d'un paquet de données

Ici, l'ESP envoie une partie du fichier image au serveur.

**Octet n°4 :** (*Entier non-signé*) Identifiant/Index du paquet.

**Octet n°5-8 :** (*Entier non-signé de 4 octets*) Taille du paquet

> Après cette demande le serveur doit repondre si il a pris connaissance du paquet ou non. Voir `protocole-serveur.md` pour ça.
  Il devra aussi enregistrer l'image
