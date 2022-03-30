/******************************************************************************
 * Pour utiliser un shield Ethernet équipé d'une w5500.                       *
 * On utilise le HSPI et on ne configure qu'un socket (pour avoir 16k de ram) *
 * en mode UDP et on configure un serveur unique avec lequel se fera la       *
 * communication. (Tout autre paquet sera ignoré.)                            *
 * Cette bibliothèque EthernetUDP.h est optimisée pour notre utilisation      *
 * specifique.                                                                *
 * Pour voir la doc de la w5500 :                                             *
 * - https://docs.wiznet.io/Product/iEthernet/W5500/overview                  *
 * - https://docs.wiznet.io/Product/iEthernet/W5500/datasheet                 *
 ******************************************************************************/

#include "EthernetUDP.h"




//----------------------------------Ethernet-----------------------------------


//-Ethernet---------------------------begin------------------------------------


boolean EthernetClass::begin(const uint8_t * mac, const uint8_t * gateway, const uint8_t * subnet, const uint8_t * ip) {
  // Enregistre dans la w5500 l'adresse mac, le gateway, le subnet et l'ip de la carte
  // l'adresse mac est un tableu de 6 uint8_t, les adresses ip sont des tableau de 4 uint8_t

  // Initialise la communication SPI avec le shield et fait les première
  // configuration (si ça fonctionne pas on arréte)
  if (!w5500.init()) return false;

  // Enregistrement
  w5500.write4Bytes(w5500.commonRegister, 0x0001, gateway);
  w5500.write4Bytes(w5500.commonRegister, 0x0005, subnet);
  w5500.writeLen(w5500.commonRegister, 0x0009, mac, 6);
  w5500.write4Bytes(w5500.commonRegister, 0x000F, ip);

  // Affectation de la mémoire RX et TX de chaque socket (8 pour les  deux
  // premiers et 0 pour les autres) (voir doc w5500)
  w5500.writeByte(w5500.socket0Register, 0x001E, 8); // 8k octets de mémoire pour buffer RX du socket 0
  w5500.writeByte(w5500.socket0Register, 0x001F, 8); // 8k octets de mémoire pour buffer TX du socket 0
  w5500.writeByte(w5500.socket1Register, 0x001E, 8); // 8k octets de mémoire pour buffer RX du socket 1
  w5500.writeByte(w5500.socket1Register, 0x001F, 8); // 8k octets de mémoire pour buffer TX du socket 1
  for (char socketnRegister = 9; socketnRegister < 32; socketnRegister += 4) {
    w5500.writeByte(socketnRegister, 0x001E, 0); // 0 octets de mémoire pour buffer RX des sockets 2 à 7
    w5500.writeByte(socketnRegister, 0x001F, 0); // 0 octets de mémoire pour buffer TX des sockets 2 à 7
  }

  // Attend que le PHY se connecte
  uint8_t count = 0;
  do {
    if (linkStatus() == LinkON) return true; // réussi
    delay(10);
  } while (count++ < 100);

  return false; // fail
}


boolean EthernetClass::begin(const uint8_t * mac, const uint8_t * ip) {
  // Fonction begin qui utilise les gateway et subnet par defaut
  // gateway : xxx.xxx.xxx.1 et subnet : 255.255.255.0)

  const uint8_t gateway[4] = {ip[0], ip[1], ip[2], 1};
  const uint8_t subnet[4] = {255, 255, 255, 0};
  return begin(mac, gateway, subnet, ip);
}

boolean EthernetClass::begin(const uint8_t * mac) {
  // Pour l'instant on donne juste une adresse ip mais après ce sera pour
  // configurer avec un serveur DHCP

  const uint8_t ip[4] = {192, 168, 10, 231};
  return begin(mac, ip);
}



//-Ethernet--------------------------maintain----------------------------------


uint8_t EthernetClass::maintain() {
  return 0;
}



//-Ethernet---------------------------status-----------------------------------


ethernetHardwareStatus EthernetClass::hardwareStatus() {
  // On tente de lire la version du shield, si on ne recoit pas 4 (la bonne
  // version) c'est que soi on ne parvient pas a faire la comunication SPI soi
  // que c'est le mauvais shield

  if (w5500.readByte(w5500.commonRegister, 0x0039) == 4) return EthernetHasHardware;
  return EthernetNoHardware;
}


ethernetLinkStatus EthernetClass::linkStatus() {
  // On regarde le bit LNK (0) du W5500 PHY Configuration Register pour voir si
  // on est connecté ou non

  if (w5500.readByte(w5500.commonRegister, 0x002E) & 0b00000001) return LinkON;
  return LinkOFF;
}


String EthernetClass::localIP() {
  // On lit l'adresse IP dans le Source IP Address Register et on la return
  // sous forme de string

  uint8_t adresseIP[4];
  w5500.read4Bytes(w5500.commonRegister, 0x000F,adresseIP);
  String ret;
  ret = String(adresseIP[0]);
  ret = String(ret + ".");
  ret = String(ret + adresseIP[1]);
  ret = String(ret + ".");
  ret = String(ret + adresseIP[2]);
  ret = String(ret + ".");
  ret = String(ret + adresseIP[3]);
  return ret;
}



//-Ethernet--------------------------problem-----------------------------------


ethernetIssue EthernetClass::checkProblem(boolean ok) {
  // Regarde si la w5500 à enregistrée un problème et si oui le retourne

  // Si on a un interrupt, on verifie le Interrupt Register
  if (!w5500.stateINT()) {
    uint8_t interruptStatus = w5500.readByte(w5500.commonRegister, 0x0015);

    // On verifie alors les problèmes possibles
    if (interruptStatus & 0b10000000) return IP_CONFLICT;
    if (interruptStatus & 0b01000000) return DESTINATION_UNREACHABLE;
  }

  // on reset alors l'interupt en mettant le Socket Interrupt Register a 0
  // sauf si on a mit false en argument
  if (ok) w5500.writeByte(w5500.commonRegister, 0x0015, 0b00000000);

  // Si il n'y a pas de problèmes
  return NO_PROBLEM;
}



//-Ethernet-----------------------declaration----------------------------------

EthernetClass Ethernet;




//------------------------------------UDP--------------------------------------


//-UDP--------------------------initialisateurs--------------------------------


UDPClass::UDPClass() {
  for (uint8_t socketN=0; socketN<8; socketN++) // Pour naviguer a travers les Socket
    if (!w5500.readByte((socketN*4)+1, 0x0003) & w5500.readByte((socketN*4)+1, 0x001E)) // Si le socket est fermé (en regardant dans les sockets registers) et si les buffer ne sont pas a zéro
      _socket = socketN;
}

UDPClass::UDPClass(uint8_t socketN) {
  _socket = socketN;
}



//-UDP--------------------------------begin------------------------------------


boolean UDPClass::begin(unsigned short port) {
  // On configure un unique socket sur la w5500 en UDP

  // On verifie que le _socket soit valide (de 0 à 7)
  if (_socket > 7) return false;

  // Configuration des adresse des Registers conrespondantes au socket
  // Sous la forme 0b(socket sur 3 bit)XX
  _socketRegister = ((_socket << 2) & 0b11100) | 0b01;
  _TXRegister = ((_socket << 2) & 0b11100) | 0b10;
  _RXRegister = ((_socket << 2) & 0b11100) | 0b11;

  // On verifie que le port n'est pas deja ouvert
  if (w5500.readByte(_socketRegister, 0x0003) | !w5500.readByte(_socketRegister, 0x001E)) return false;

  // Configuration (voir doc w5500)
  w5500.writeByte(_socketRegister, 0x0000, 0b00000010); // Configuration du socket n en mode UDP dans le Socket Mode Register
  w5500.writeShort(_socketRegister, 0x0004, port); // Ouverture du port associé au socket n
  w5500.writeByte(_socketRegister, 0x0001, 0x01); // Ouverture du socket
  w5500.writeByte(_socketRegister, 0x02C, 0b00100); // Configuration du Socket n Interrupt Mask Register pour ne garder que les interrupt RECV
  w5500.writeShort(_socketRegister, 0x0012, MTU); // Ecriture du MTU dans le Socket n Maximum Segment Size Register

  // Ajout du socket parmi les interrupt possibles
  uint8_t intStatus = w5500.readByte(w5500.commonRegister, 0x0018); // On regarde le statut des interrupt
  intStatus |= (1 << _socket);
  w5500.writeByte(w5500.commonRegister, 0x0018, intStatus); // Activation de l'interrupt pour le socket 0 et  1 seul (car c'est les seuls qu'on utilisent)

  //Enregistrement du port utilisé
  _udpPort = port;

  // Verification avant de terminer
  char count = 0;
  do {
    if (w5500.readByte(_socketRegister, 0x0003) == 0x22) return true; // On test si l'ouverture est réussi sur le Socket Status Segister
    delay(2);
  } while (count++ < 20);

  // Si c'est raté
  w5500.writeByte(_socketRegister, 0x0001, 0x10); // Fermeture du socket
  return false; // Raté
}



//-UDP----------------------------configServer---------------------------------


void UDPClass::configServer(unsigned short portServeur, const uint8_t * ipServeur) {
  // Configure notre destination et port (communication avec le même serveur)

  // Configuration du port de destination
  w5500.writeShort(_socketRegister, 0x0010, portServeur);

  // Configuration de l'ip de destination
  w5500.write4Bytes(_socketRegister, 0x000C, ipServeur);

  //Enregistrement du port et de l'adresse IP de destination
  _portServeur = portServeur;
  for (uint8_t i=0; i<4; i++) _ipServeur[i] = ipServeur[i];
}



//-UDP----------------------------parsePacket----------------------------------


uint16_t UDPClass::parsePacket() {
  // Regarde si il y'a un paquet et retourne sa taille ou 0 si non
  // Ne process que le prochain paquet
  // Si le buffer est vide, elle enlève l'interrupt

  // On met le pointeur au niveau du prochain paquet en prenant la position de
  // notre pointeur de Lecture dans le Socket n RX Read Data Pointer Register
  // et en l'incrementant du reste
  if (_octetsRestant) {
    uint16_t pointeurLecture = w5500.readShort(_socketRegister, 0x0028);
    w5500.writeShort(_socketRegister, 0x0028,(uint16_t) pointeurLecture+_octetsRestant);
    w5500.writeByte(_socketRegister, 0x0001, 0x40); // Commande RECV
    _octetsRestant=0;
  }

  // On regarde si le pin INT a bien été assert a LOW et on verifie ensuite
  // que ce soit bel et bien une reception de paquet en demandant à la puce
  // la taille du paquet
  if (w5500.stateINT()) {

    // On regarde la longueur dans le Socket n Received Size Register
    // (voir doc w5500 pour comprendre pourquoi on le fait plusieurs fois)
    // (Ca fonctionne
    uint16_t remplissageBuffer, test;
    remplissageBuffer = w5500.readShort(_socketRegister, 0x0026);
    do {
      test = remplissageBuffer;
      remplissageBuffer = w5500.readShort(_socketRegister, 0x0026);
    } while(remplissageBuffer != test);
    // TODO : ça fonctionne pas je sais pas pk, bizzare (en fait si)

    if (remplissageBuffer > 0) {

      // On prend la position de notre pointeur de Lecture dans le Socket n RX
      // Read Data Pointer Register
      uint16_t pointeurLecture = w5500.readShort(_socketRegister, 0x0028);

      // On recupere le header UDP (8 premiers octets) depuis le buffer RX pour
      // trouver l'IP et le port de provenance, ainsi que la taille du paquet
      uint8_t provenanceIp[4];
      w5500.read4Bytes(_RXRegister, pointeurLecture, provenanceIp);
      uint16_t provenancePort = w5500.readShort(_RXRegister, pointeurLecture+4);
      uint16_t taillePaquet = w5500.readShort(_RXRegister, pointeurLecture+6);

      // On incremente le pointeur de lecture de la taille du header
      pointeurLecture += 8;

      // On verifie que le paquet provient du bon serveur en comparant l'ip et
      // le port pour savoir si on l'ignore ou non
      // (que si on a defini VERIFY_PROVENANCE)
      #ifdef VERIFY_PROVENANCE
      boolean ignore = false;

      for (uint8_t i=0; i<4; i++) {
        if (provenanceIp[i] != _ipServeur[i]) {ignore=true;break;}
      }
      #ifdef VERIFY_PORT_PROVENANCE
      if (provenancePort != _portServeur) ignore=true;
      #endif

      // Si on l'ignore, alors on incremente le pointeur RX de la taille du paquet
      if (ignore) pointeurLecture+=taillePaquet;
      #endif

      // On met à jour le pointeur de lecture RX dans le Socket n RX Read Data
      // Pointer Register et on envoie la commande RECV dans le Socket n
      // Command Register pour dire la w5500 qu'on a lu une partie du buffer RX
      w5500.writeShort(_socketRegister, 0x0028,(uint16_t) pointeurLecture);
      w5500.writeByte(_socketRegister, 0x0001, 0x40); // Commande RECV

      #ifdef VERIFY_PROVENANCE
      // Si on l'ignore, alors on retourne une taille nulle
      if (ignore) return 0;
      #endif


      // Tout les tests ont été fait, on stocke et on return la taille du paquet
      _octetsRestant = taillePaquet;
      return taillePaquet;

    }
    else if (Ethernet.checkProblem(false)) return 0; // On est dans un cas ou il y'a une erreur

    // Ca veut dire que le remplissageBuffer est a zéro et qu'il n'y a pas
    // d'erreur, on reset alors l'interupt en mettant le bit RECV du Socket 0
    // Interrupt Register a 1
    else w5500.writeByte(_socketRegister, 0x0002, 0b00100);

  }
  return 0; // Il n'y a pas de paquet
}



//-UDP-------------------------------read--------------------------------------


uint8_t UDPClass::read() {

  // On verifie qu'il reste encore des caractères à lire
  if (!_octetsRestant) return 0;

  // On prend la position de notre pointeur de Lecture dans le Socket 0 RX
  // Read Data Pointer Register
  uint16_t pointeurLecture = w5500.readShort(_socketRegister, 0x0028);

  // On enregistre le prochain caractère depuis le buffer RX
  uint8_t lecture = w5500.readByte(_RXRegister, pointeurLecture);

  // On met à jour le pointeur de lecture RX dans le Socket n RX Read Data
  // Pointer Register et on envoie la commande RECV dans le Socket n
  // Command Register pour dire la w5500 qu'on a lu une partie du buffer RX
  w5500.writeShort(_socketRegister, 0x0028, pointeurLecture+1);
  w5500.writeByte(_socketRegister, 0x0001, 0x40); // Commande RECV

  // On met a jour le _octetsRestant et on retourne le caractère lu
  _octetsRestant--;
  return lecture;
}

uint16_t UDPClass::read(uint8_t * buffer, uint16_t len) {

  // On verifie que len ne depasse pas le nombre de caractères restants et si
  // oui on le bloque a cette valeur
  if (len > _octetsRestant) len = _octetsRestant;

  // On prend la position de notre pointeur de Lecture dans le Socket 0 RX
  // Read Data Pointer Register
  uint16_t pointeurLecture = w5500.readShort(_socketRegister, 0x0028);

  // On place dans le buffer les prochains len caractères
  w5500.readLen(_RXRegister, pointeurLecture, buffer, len);

  // On met à jour le pointeur de lecture RX dans le Socket n RX Read Data
  // Pointer Register et on envoie la commande RECV dans le Socket n
  // Command Register pour dire la w5500 qu'on a lu une partie du buffer RX
  w5500.writeShort(_socketRegister, 0x0028, pointeurLecture+len);
  w5500.writeByte(_socketRegister, 0x0001, 0x40); // Commande RECV

  // On met a jour le _octetsRestant et on retourne le caractère la longueur
  // qu'on a réellemnt lue
  _octetsRestant -= len;
  return len;
}



//-UDP---------------------------beginPacket-----------------------------------


boolean UDPClass::beginPacket() {
  // Initialise l'envoi en remettant à zero les variables et le pointeur au cas
  // où on aurait écrit dans le buffer TX entre temps

  if (_octetsEcrits) { // Si on avait écrit

    // On met _octetsEcrits à 0
    _octetsEcrits = 0;

  }
  return true;
}



  //-UDP-----------------------------write-------------------------------------


boolean UDPClass::write(uint8_t octet) {
  // Verifie si il y a la place dans le buffer TX et si on ne depasse pas le
  // MTU et retourne false sinon. Si il y a la place, ajoute un octet au buffer
  // TX

  // Check la place restante dans le Socket n TX Free Size Register et retourne
  // false si ce n'est pas possible d'ecrire (pas la place) ou si on depasse le
  // MTU (voir doc w5500 pour comprendre pourquoi on le fait plusieurs fois)
  uint16_t placeRestante, test;
  placeRestante = w5500.readShort(_socketRegister, 0x0020);
  do {
    test = placeRestante;
    placeRestante = w5500.readShort(_socketRegister, 0x0026);
  } while(placeRestante != test);

  //if (_octetsEcrits >= MTU || placeRestante == _octetsEcrits) return false;

  // On prend la position de notre pointeur d'Ecriture dans le Socket n TX
  // Write Data Pointer Register
  uint16_t pointeurEcriture = w5500.readShort(_socketRegister, 0x0024);

  // On on ecrit l'octet dans le buffer TX à la suite
  w5500.writeByte(_TXRegister, pointeurEcriture+_octetsEcrits, octet);

  //On met à jour _octetsEcrits et on retourne la reussite (true)
  _octetsEcrits++;
  return true;
}


boolean UDPClass::write(const uint8_t * buffer, uint16_t len) {
  // Verifie si il y a la place dans le buffer TX et retourne false sinon
  // Si il y a la place, ajoute len octet (max MTU) du buffer 'buffer' au
  // buffer TX

  // Check la place restante dans le Socket n TX Free Size Register et retourne
  // false si ce n'est pas possible d'ecrire (pas la place)
  // (voir doc w5500 pour comprendre pourquoi on le fait plusieurs fois)
  uint16_t placeRestante, test;
  placeRestante = w5500.readShort(_socketRegister, 0x0020);
  do {
    test = placeRestante;
    placeRestante = w5500.readShort(_socketRegister, 0x0026);
  } while(placeRestante != test);

  //if ((_octetsEcrits+len) >= MTU || placeRestante < (_octetsEcrits+len)) return false;
  // Ca veut pas fonctionner, on dirait que le S0_FSR renvoit toujours 0

  // On prend la position de notre pointeur d'Ecriture dans le Socket n TX
  // Write Data Pointer Register
  uint16_t pointeurEcriture = w5500.readShort(_socketRegister, 0x0024);

  // On on ecrit l'octet dans le buffer TX à la suite
  w5500.writeLen(_TXRegister, pointeurEcriture+_octetsEcrits, buffer, len);

  //On met à jour _octetsEcrits et on retourne la reussite (true)
  _octetsEcrits += len;
  return true;
}



//-UDP----------------------------endPacket------------------------------------


boolean UDPClass::endPacket() {
  // Demande l'envoi du paquet au serveur specifié dans configServeur et
  // verifie si l'envoi à réussi

  // On prend la position de notre pointeur d'Ecriture dans le Socket n TX
  // Write Data Pointer Register
  uint16_t pointeurEcriture = w5500.readShort(_socketRegister, 0x0024);
  // On met à jour le pointeur d'écriture TX dans le Socket n TX Write Data
  // Pointer Register
  w5500.writeShort(_socketRegister, 0x0024, pointeurEcriture+_octetsEcrits);

  // Envoi la commande SEND au Socket n Command Register
  w5500.writeByte(_socketRegister, 0x0001, 0x0020);

  // On regarde le statut de l'interrupt dans le Socket n Interrupt Register et
  // on attend que le bit SEND_OK soit à 1 tout en verifiant que le bit
  // TIMEOUT ne le soit pas
  uint8_t interruptStatus;
  do {
    interruptStatus = w5500.readByte(_socketRegister, 0x0002); // Check le statut de l'interrupt

    if (interruptStatus & 0b01000) { // En cas de timeout
      // On indique qu'on a pris connaissance de l'info en ecrivant 1 sur le
      // bit TIMEOUT du Socket n Interrupt Register puis on return false
      w5500.writeByte(_socketRegister, 0x0002, 0b01000);
      return false;
    }
  } while (!(interruptStatus & 0b10000));

  // On indique qu'on a pris connaissance de l'info en ecrivant 1 sur le
  // bit SEND_OK du Socket n Interrupt Register puis on met _octetsEcrits à 0
  // et on return true
  w5500.writeByte(_socketRegister, 0x0002, 0b10000);
  _octetsEcrits = 0;
  return true;

}



//-UDP------------------------------close--------------------------------------


void UDPClass::close() {
  // Fermeture du socket et clear des interrupt

  // Clear du Socket n Interrupt Register en ecrivant 1 partout
  w5500.writeByte(_socketRegister, 0x0002, 0b11111111);

  // TODO : enlever le mask interrupt de ce socket

  // Clear de l'Interrupt Register en ecrivant 1 partout
  w5500.writeByte(w5500.commonRegister, 0x0015, 0b11111111);

  // Envoi de la commande CLOSE au Socket n Command Register
  w5500.writeByte(_socketRegister, 0x0001, 0x10);
}


UDPClass::~UDPClass() {
  // Le destructeur ferme le socket avant
  close();
}
