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

  // Enregistrement dans la w5500
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

  // Mise à zéro des Interrupt Mask Registers
  w5500.writeByte(w5500.commonRegister, 0x0018, 0b00000000);

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
  // Configure le client en DHCP

  // On initialise une classe DHCP
  _dhcp = new DHCPClass();

  // On commence l'ethernet avec une ip par défaut 0.0.0.0
  if (!begin(mac, _dhcp->ipAddr)) return false;

  // On demarre le DHCP
  _dhcp->openSocket();

  // On lance l'allocation DHCP
  if (!_dhcp->allocation(mac)) return false;

  // On enregistre les nouvelles adresses dans la w5500
  w5500.write4Bytes(w5500.commonRegister, 0x0001, _dhcp->gatewayAddr);
  w5500.write4Bytes(w5500.commonRegister, 0x0005, _dhcp->subnetAddr);
  w5500.writeLen(w5500.commonRegister, 0x0009, _dhcp->macAddr, 6);
  w5500.write4Bytes(w5500.commonRegister, 0x000F, _dhcp->ipAddr);

  return true; // Reussi
}



//-Ethernet--------------------------maintain----------------------------------


uint8_t EthernetClass::maintain() {
  // S'occupe du renouvellement du bail DHCP
  // Retourne :
  // - 0 si rien ne se passe
  // - 1 en cas d'echec du renouvelement du bail
  // - 2 en cas de renouvellement du bail
  // - 3 en cas d'echec de la demande d'un nouveau bail
  // - 4 en cas de demande d'un nouveau bail

  // On regarde si on est en configuration DHCP
  if (!_dhcp) return 0;

  // On a alors trois cas possible, soi on est dans la la première moitié du
  // temps alloué au bail DHCP et dans ce cas la rien ne se passe, soi on est
  // dans la seconde moitié du temps alloué pour le bail DHCP et dans ce cas
  // mieux vaut tenter de renouveller le bail. La derniere possibillité
  // (qui ne devrait pas arriver normalement) est que le bail est expiré. On
  // recommence alors la configuration DHCP depuis le debut.

  // On calcule le temps passé depuis le debut du bail
  unsigned long timeSinceLease = (millis() / 1000) - _dhcp->timeAtOffer;

  // Dans le cas où on est dans la première moitiédu temps imparti
  // On ne fait rien
  if (timeSinceLease < (_dhcp->ipAddrLeaseTime/2)) return 0;

  // Dans le cas où on a dépassé le temps du bail
  // (On doit recommencer)
  if (timeSinceLease > _dhcp->ipAddrLeaseTime) {

    // On remet à 0 l'ip
    memset(_dhcp->ipAddr, 0, 4);

    // On tente une reallocation, si elle foire on return 3
    if (!_dhcp->allocation(_dhcp->macAddr)) return 3;

    // Si elle reussi on enregistre les nouvelles adresses dans la w5500
    w5500.write4Bytes(w5500.commonRegister, 0x0001, _dhcp->gatewayAddr);
    w5500.write4Bytes(w5500.commonRegister, 0x0005, _dhcp->subnetAddr);
    w5500.writeLen(w5500.commonRegister, 0x0009, _dhcp->macAddr, 6);
    w5500.write4Bytes(w5500.commonRegister, 0x000F, _dhcp->ipAddr);

    // Et on return 4
    return 4;
  }

  // Dans le cas où on est dans la seconde moitié du temps imparti
  // On tente de renouveller le bail
  else {
    if (_dhcp->renewal()) return 2; // Si ca reussi on return 2
    // Si ça foire on met le temps de bail à 0 pour forcer à refaire une
    // allocation complète la prochaine fois et on return 1
    else {
      _dhcp->ipAddrLeaseTime = 0;
      return 1;
    }
  }
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



//-Ethernet----------------------------end-------------------------------------


void EthernetClass::end() {
  // Termine les operations de l'Ethernet (surtout pour le DHCP)

  // On supprime la classe DHCP et on libere l'addrese IP si on était en mode
  // DHCP
  if (_dhcp) {
    _dhcp->release();
    delete _dhcp;
    _dhcp = NULL;
  }
}



//-Ethernet-----------------------declaration----------------------------------

EthernetClass Ethernet;




//------------------------------------UDP--------------------------------------


//-UDP--------------------------initialisateurs--------------------------------


UDPClass::UDPClass() {
  _socket = 8;
}

UDPClass::UDPClass(uint8_t socketN) {
  _socket = socketN;
}



//-UDP--------------------------------begin------------------------------------


boolean UDPClass::begin(unsigned short port) {
  // On configure un unique socket sur la w5500 en UDP

  // Si le socket n'est pas valide (de 1 à 7), on en cherche un nouveau
  if (_socket > 7) {
    for (uint8_t socketN=0; socketN<8; socketN++) { // Pour naviguer a travers les Socket
      if (w5500.readByte((socketN*4)+1, 0x0003) == 0x00 && w5500.readByte((socketN*4)+1, 0x001E) != 0) { // Si le socket est fermé (en regardant dans les sockets registers) et si les buffer ne sont pas a zéro
        _socket = socketN;
        break;
      }
    }
  }

  // Pas de socket a été trouvé
  if (_socket > 7) return false;

  //Serial.println("Socket : " + String(_socket));

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
  w5500.writeByte(w5500.commonRegister, 0x0018, intStatus); // Activation de l'interrupt

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


uint16_t UDPClass::parsePacket(boolean all) {
  // Regarde si il y'a un paquet et retourne sa taille ou 0 si non
  // Ne process que le prochain paquet
  // Si le buffer est vide, elle enlève l'interrupt
  // Si all = true, la provenance ne sera pas verifiée

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

      if (!all) { // Sauf si on laisse passer tout les paquets
        for (uint8_t i=0; i<4; i++) {
          if (provenanceIp[i] != _ipServeur[i]) {ignore=true;break;}
        }
        #ifdef VERIFY_PORT_PROVENANCE
        if (provenancePort != _portServeur) ignore=true;
        #endif

        // Si on l'ignore, alors on incremente le pointeur RX de la taille du paquet
        if (ignore) pointeurLecture+=taillePaquet;
      }
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
    // d'erreur, on reset alors l'interupt en mettant le bit RECV du Socket n
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

  // Enlevage (non ça se dit pas) du socket parmi les interrupt possibles
  uint8_t intStatus = w5500.readByte(w5500.commonRegister, 0x0018); // On regarde le statut des interrupt
  intStatus &= ~(1 << _socket);
  w5500.writeByte(w5500.commonRegister, 0x0018, intStatus); // Activation de l'interrupt

  // Clear de l'Interrupt Register en ecrivant 1 partout
  w5500.writeByte(w5500.commonRegister, 0x0015, 0b11111111);

  // Envoi de la commande CLOSE au Socket n Command Register
  w5500.writeByte(_socketRegister, 0x0001, 0x10);
}


UDPClass::~UDPClass() {
  // Le destructeur ferme le socket avant
  close();
}




//-----------------------------------DHCP--------------------------------------


//-DHCP-------------------Constructeur/Destructeurs----------------------------


DHCPClass::DHCPClass() {
// Nada
}


DHCPClass::~DHCPClass() {
  // Declenche la liberation de l'adresse ip si il y en avait une
  if (ipAddrLeaseTime) release();
}



//-DHCP-----------------------------Begin--------------------------------------


boolean DHCPClass::openSocket() {
  // Après la création de la classe DHCP, on initialise notre socket UDP avec
  // les ports DHCP et l'ip serveur 255.255.255.255 (pour le broadcast)
  // /!\ Doit être appelé avant toute fonction DHCP
  if (!_udpDHCP.begin(68)) return false; // Pas reussi
  uint8_t broadcast[4] = {255, 255, 255, 255};
  _udpDHCP.configServer(67, broadcast);
  return true; // Reussi
}



//-DHCP---------------------------Messages-------------------------------------


// Pour comprendre les trames DHCP (dont RFC 2131 et 2132):
//   - https://www.netmanias.com/en/post/techdocs/5998/dhcp-network-protocol
//     /understanding-the-basic-operations-of-dhcp
//   - http://abcdrfc.free.fr/rfc-vf/pdf/rfc2132.pdf
//   - https://datatracker.ietf.org/doc/html/rfc2131
//   - https://datatracker.ietf.org/doc/html/rfc2132


void DHCPClass::_sendDiscover() {
  // Construit et envoi un paquet DHCP Discover
  // /!\ Le serveur doit être configuré avec une ip broadcast

  // Initialisation de la trame qu'on enverra pour le DHCP DISCOVER
  uint8_t trame[257];
  memset(trame, 0, 257);

  // Rempli les infos génerales de la trame
  _remplissageEntete(trame);

  // -- trame[240 à 242] --
  // DHCP Message Type (Option 53) -> DHCP Discover (1)
  trame[240] = 53; // Code (Option 53)
  trame[241] = 1; // Longueur (1 bit)
  trame[242] = 1; // Type (DHCP Discover)

  // -- trame[243 à 251] --
  // Client - Iddentifier (Option 61) -> Addresse MAC
  trame[243] = 61; // Code (Option 61)
  trame[244] = 7; // Longueur (7 bit)
  trame[245] = 1; // Type (Ethernet : 1)
  memcpy(trame+246, macAddr, 6); // Adress MAC

  // -- trame[252 à 255] --
  // Parameter Request List (Option 55) -> Subnet (1), Gateway (3)
  trame[252] = 55; // Code (Option 55)
  trame[253] = 2; // Longueur (2 bit)
  trame[254] = 1; // Parametre Subnet
  trame[255] = 3; // Parametre Gateway

  // -- trame[256] --
  // Fin des options
  trame[256] = 255; // Code Bit de Fin

  // Envoi de la requette
  _udpDHCP.beginPacket();
  _udpDHCP.write(trame, 257);
  _udpDHCP.endPacket();
}


boolean DHCPClass::_recvOffer() {
  // Regarde si une offre est recue, si oui elle stocke l'ip offerte dans
  // _offeredIpAddr et return true, sinon elle return false

  // On regarde les paquets entrants
  unsigned short longueur = _udpDHCP.parsePacket(true);
  if (longueur) { // Si on a reçu quelque chose

    if (longueur < 236) return false; // Si le paquet est trop petit c'est pas la peine

    // Creation d'un buffer de reception
    uint8_t * trame;
    trame = (uint8_t *)malloc(longueur);

    // On copie le paquet dans le buffer
    _udpDHCP.read(trame, longueur);


    // ------ Analyse du paquet et verification ------
    // On verifie que le paquet nous est bien déstiné et on enregitre les
    // données à enregistrer

    // On verifie les données de l'entete
    if (!_verificationEntete(trame)) {free(trame); return false;}


    // Parcours des options :
    unsigned short i = 240; // Pointeur de parcours (on le met au debut du champs option)
    while (trame[i] != 255) { // Tant qu'on tombe pas sur le paquet de fin
      switch (trame[i]) {

        // Parametre Pad (Option 0) (a skiper)
        case 0 : i++; break;

        // Parametre DHCP Message Type (Option 53) (on verifie que ce soit bien à 2 [DHCP Offer])
        case 53 :
          if (trame[i+2] != 2) {free(trame); return false;}
          i += 3; break;

        // Server identifier (Option 54) (On l'enregistre dans _ipAddrServerDHCP)
        case 54 :
          memcpy(_ipAddrServerDHCP, trame+i+2, 4);
          i += 6; break;

        // Autre Paramètre (on skip de la longueur du parametre)
        default : i += trame[i+1]+2; break;
      }
    }

    // -- trame[16 à 19] --
    // Your IP Address (yiaddr) -> Enregistrement de l'adresse proposée
    memcpy(_offeredIpAddr, trame+16, 4);

    // Fin reussite
    free(trame);
    return true;
  }

  // Si on a rien reçu
  else return false;
}


void DHCPClass::_sendRequest(boolean renew) {
  // Construit et envoi un paquet DHCP Request avec l'adresse _offeredIpAddr
  // /!\ Le serveur reste en mode Broadcast
  // En cas de renewal, on met renew en true, la requette ne donnera alors pas
  // la Requested IP Address (Option 50) et et la DHCP Server Identifier
  // (Option 54) car les serveurs le savent deja

  // Initialisation de la trame qu'on enverra pour le DHCP REQUEST
  uint8_t trame[269];
  memset(trame, 0, 269);

  // Rempli les infos génerales de la trame
  _remplissageEntete(trame);

  // -- trame[240 à 242] --
  // DHCP Message Type (Option 53) -> DHCP Request (3)
  trame[240] = 53; // Code (Option 53)
  trame[241] = 1; // Longueur (1 bit)
  trame[242] = 3; // Type (DHCP Request)

  // -- trame[243 à 251] --
  // Client - Iddentifier (Option 61) -> Addresse MAC
  trame[243] = 61; // Code (Option 61)
  trame[244] = 7; // Longueur (6 bit)
  trame[245] = 1; // Type (Ethernet : 1)
  memcpy(trame+246, macAddr, 6); // Adress MAC

  // Si on fait la requette lors d'une allocation, elle sera plus longue
  // on devra donc ajouter 'add' à sa taille
  unsigned char add = 0;

  if (!renew) {
    // -- trame[252 à 257] --
    // Requested IP Adresses (Option 50) -> _offeredIpAddr
    trame[252] = 50; // Code (Option 50)
    trame[253] = 4; // Longueur (4 bit)
    memcpy(trame+254, _offeredIpAddr, 4); // Adresse offerte

    // -- trame[258 à 263] --
    // DHCP Server Identifier (Option 54) -> _ipAddrServerDHCP
    trame[258] = 54; // Code (Option 54)
    trame[259] = 4; // Longueur (4 bit)
    memcpy(trame+260, _ipAddrServerDHCP, 4); // Adresse du serveur qui à fait l'offre

    // On ajoutera donc 12 de plus à chaque fois qu'on veut écrire derrière
    add = 12;
  }

  // -- trame[264 à 265] -- (ou [252 à 255])
  // Parameter Request List (Option 55) -> Subnet (1), Gateway (3)
  trame[252+add] = 55; // Code (Option 55)
  trame[253+add] = 2; // Longueur (2 bit)
  trame[254+add] = 1; // Parametre Subnet
  trame[255+add] = 3; // Parametre Gateway

  // -- trame[268] -- (ou [256])
  // Fin des options
  trame[256+add] = 255; // Code Bit de Fin

  // Envoi de la requette
  _udpDHCP.beginPacket();
  _udpDHCP.write(trame, 257+add);
  _udpDHCP.endPacket();
}


boolean DHCPClass::_recvAck() {
  // Regarde si le serveur accepte, si oui elle stocke les infos dans les
  // variables publiques (ipAddr, gatewayAddr, subnetAddr, ipAddrLeaseTime,
  // timeAtOffer) et sinon elle return false

  // On regarde les paquets entrants
  unsigned short longueur = _udpDHCP.parsePacket(true);
  if (longueur) { // Si on a reçu quelque chose

    if (longueur < 236) return false; // Si le paquet est trop petit c'est pas la peine

    // Creation d'un buffer de reception
    uint8_t * trame;
    trame = (uint8_t *)malloc(longueur);

    // On copie le paquet dans le buffer
    _udpDHCP.read(trame, longueur);


    // ------ Analyse du paquet et verification ------
    // On verifie que le paquet nous est bien déstiné et on enregitre les
    // données à enregistrer

    // On verifie les données de l'entete
    if (!_verificationEntete(trame)) {free(trame); return false;};


    // Parcours des options :
    unsigned short i = 240; // Pointeur de parcours (on le met au debut du champs option)
    while (trame[i] != 255) { // Tant qu'on tombe pas sur le paquet de fin
      switch (trame[i]) {

        // Parametre Pad (Option 0) (a skiper)
        case 0 : i++; break;

        // Parametre DHCP Message Type (Option 53) (on verifie que ce soit bien à 5 [DHCP Ack])
        // Cela supprimera aussi les paquets NACK
        case 53 :
          if (trame[i+2] != 5) {free(trame); return false;}
          i += 3; break;

        // Subnet Mask (Option 1) (on l'enregistre)
        case 1 :
          memcpy(subnetAddr, trame+i+2, 4);
          i += 6; break;

        // Router IP (Option 3) (on l'enregistre)
        case 3 :
          memcpy(gatewayAddr, trame+i+2, 4);
          i += 6; break;

        // IP Adress Lease Time (Option 51) (on l'enregistre)
        case 51 :
          uint32_t time;
          memcpy(&time, trame+i+2, 4);
          ipAddrLeaseTime = ntohl(time);
          i += 6; break;

        // Server identifier (Option 54) (On l'enregistre dans _ipAddrServerDHCP)
        // On devrait verifier mais faudrait le faire avant d'enregistrer donc
        // flemme, trop compliqué
        case 54 :
          memcpy(_ipAddrServerDHCP, trame+i+2, 4);
          i += 6; break;

        // Autre Paramètre (ont skip de la longueur du parametre)
        default : i += trame[i+1]+2; break;

      }
    }

    // -- trame[16 à 19] --
    // Your IP Address (yiaddr) -> Verification et enregistrement de l'adresse
    for (uint8_t i=0; i<4; i++) if (trame[16+i] != _offeredIpAddr[i]) {free(trame); return false;};
    memcpy(ipAddr, trame+16, 4);

    // Enregistrement du temps qu'il est
    timeAtOffer = millis() / 1000;

    // Fin reussite
    free(trame);
    return true;
  }

  // Si on a rien reçu
  else return false;
}


void DHCPClass::_sendRelease() {
  // Construit et envoi un paquet DHCP Release avec l'adresse ipAddr

  // Initialisation de la trame qu'on enverra pour le DHCP RELEASE
  uint8_t trame[257];
  memset(trame, 0, 257);


  // Rempli les infos génerales de la trame
  _remplissageEntete(trame);

  // -- trame[240 à 242] --
  // DHCP Message Type (Option 53) -> DHCP Release (7)
  trame[240] = 53; // Code (Option 53)
  trame[241] = 1; // Longueur (1 bit)
  trame[242] = 7; // Type (DHCP release)

  // -- trame[243 à 251] --
  // Client - Iddentifier (Option 61) -> Addresse MAC
  trame[243] = 61; // Code (Option 61)
  trame[244] = 7; // Longueur (6 bit)
  trame[245] = 1; // Type (Ethernet : 1)
  memcpy(trame+246, macAddr, 6); // Adress MAC

  // -- trame[252 à 255] --
  // Parameter Request List (Option 55) -> Subnet (1), Gateway (3)
  trame[252] = 55; // Code (Option 55)
  trame[253] = 2; // Longueur (2 bit)
  trame[254] = 1; // Parametre Subnet
  trame[255] = 3; // Parametre Gateway

  // -- trame[256] --
  // Fin des options
  trame[256] = 255; // Code Bit de Fin

  // Envoi de la requette
  _udpDHCP.beginPacket();
  _udpDHCP.write(trame, 257);
  _udpDHCP.endPacket();
}



//-DHCP---------------Aide-Ercriture/Lecture-Messages--------------------------


void DHCPClass::_remplissageEntete(uint8_t * trame) {
  // Remplis le debut d'une trame passée en entrée, si plus de modifications
  // sont à faire sur l'entete, elle seront faite dans la fonction
  // correspondante
  // La trame doit etre d'une taille suffisante


  // -- trame[0 à 3] --
  // Entete
  trame[0] = 1; // OP Code -> Requette (1)
  trame[1] = 1; // Hardware Type -> Ethernet (1)
  trame[2] = 6; // Hardware Len (Taille de l'adresse materielle [MAC]) -> 6
  // trame[3] = 0; // Hops Count (Le client doit le mettre à zéro, c'est pour les relay agent)

  // -- trame [4 à 7] --
  // Identifiant de la transaction (generé aleatoirement au debut de celle ci)
  uint32_t xid = htonl(_transactionID);
  memcpy(trame+4, &xid, sizeof(uint32_t));

  // -- trame[8 et 9] --
  // Nombre de secondes écoulées depuis le debut de l'allocation
  uint16_t secondSinceBegin = (millis() - _beginTime) / 1000;
  secondSinceBegin = htons(secondSinceBegin);
  memcpy(trame+8, &secondSinceBegin, sizeof(uint16_t));

  // -- trame[10 et 11] --
  // Bit de broadcast (B) (MSB de trame[10]) -> 1 car on veut que le serveur
  // nous unicast le DHCP OFFER/ACK a notre adresse MAC
  // Flags (Restes des bits de trame[10 et 11]) -> 000000000000000 car doivent
  // être laissés à 0
  // 0b 0 000000000000000
  //   |B|    Flags     |

  // -- trame[12 à 15] --
  // Client IP Address (ciaddr) -> Notre adresse IP (quand on en a pas elle
  // est de 0.0.0.0)
  memcpy(trame+12, ipAddr, 4);

  // -- trame[16 à 19] --
  // Your IP Address (yiaddr) -> 0.0.0.0 car on ne connait pas

  // -- trame[20 à 23] --
  // Server IP Address (siaddr) -> 0.0.0.0 car on ne connait pas

  // -- trame[24 à 27] --
  // Gateway IP Address (giaddr) -> 0.0.0.0 car on ne connait pas

  // -- trame[28 à 43] --
  // Client Hardware Address (chaddr) -> Notre adresse MAC
  memcpy(trame+28, macAddr, 6);

  // -- trame[44 à 107] --
  // Server Name -> On met tout à zéro

  // -- trame[108 à 235] --
  // Filename -> On met tout à zéro

  // -- trame [236 à 239] --
  // Magic Cookie (indique que c'est un paquet DHCP et non BOOTP)
  trame[236] = 99;
  trame[237] = 130;
  trame[238] = 83;
  trame[239] = 99;
}


boolean DHCPClass::_verificationEntete(const uint8_t * trame) {
  // Verifie l'entete du paquet pour verifier qu'il nous est bien destiné
  // Fait les verifications repetitives pour eviter d'avoir un code répétitif
  // Si plus de choses sont à enregistrer ou verifier, ce sera fait dans la
  // fonction correspondante
  // La trame doit etre d'une taille suffisante

  // ------ Analyse du paquet et verification ------
  // On verifie que le paquet nous est bien déstiné

  // -- trame[0] --
  // OP Code -> Reply (2)
  if (trame[0] != 2) return false;

  // -- trame[2] --
  // Hardware Length -> Taille de l'adresse MAC (6)
  if (trame[2] != 6) return false;

  // -- trame[4 à 7] --
  // Transaction ID -> _transactionID
  uint32_t receivedTransactionID;
  memcpy(&receivedTransactionID, trame+4, 4);
  receivedTransactionID = ntohl(receivedTransactionID);
  if (receivedTransactionID != _transactionID) return false;

  // -- trame[12 à 15] --
  // Client IP Address (ciaddr) -> Notre adresse IP (quand on en a pas elle est
  // de 0.0.0.0)
  for (uint8_t i=0; i<4; i++) if (trame[12+i] != ipAddr[i]) return false;

  // -- trame[28 à 43] --
  // Client Hardware Address (chaddr) -> Notre adresse MAC
  for (uint8_t i=0; i<6; i++) if (trame[28+i] != macAddr[i]) return false;

  // -- trame [236 à 239] --
  // On verifie le Magic Cookie (qui indique que c'est un paquet DHCP et non
  // BOOTP)
  if (trame[236] != 99) return false;
  if (trame[237] != 130) return false;
  if (trame[238] != 83) return false;
  if (trame[239] != 99) return false;

  // Tout est bon
  return true;
}



//-DHCP--------------------------Allocation------------------------------------


boolean DHCPClass::allocation(const uint8_t * mac) {
  // S'occupe du processus d'allocation et stocke les infos dans les
  // variables publiques

  // Mise en place des variables pour l'allocation
  _beginTime = millis();
  _transactionID = micros()*12345; // Nombre aleatoire pour la transaction

  // Mise en place de l'adresse MAC
  memcpy(macAddr, mac, 6);

  // La w5500 a du mal à commencer ses envois tout de suite jsp pk mais quand
  // on attend un peu ça a l'air de regler le problème
  delay(500);

  // Envoi de DHCP DISCOVER
  _sendDiscover();

  // Reception du paquet DHCP OFFER (et enregistrement de l'adresse dans
  // _offeredIpAddr) + Gestion du Timeout
  while (!_recvOffer()) {
    if (millis() - _beginTime > DHCP_TIMEOUT) return false;
  }

  // Envoi de DHCP REQUEST avec l'adresse ip dans _offeredIpAddr
  _sendRequest();

  // Reception du paquet DHCP ACK (et enregistrement du reste des adresses
  // et variables) + Gestion du Timeout
  while (!_recvAck()) {
    if (millis() - _beginTime > DHCP_TIMEOUT) return false;
  }

  // Reussite
  return true;
}



//-DHCP--------------------------Renewal------------------------------------


boolean DHCPClass::renewal() {
  // S'occupe du processus de renouvellement du bail et avec les infos des
  // variables publiques

  // Remise à zéro du temps
  _beginTime = millis();

  // Envoi de DHCP REQUEST avec l'adresse ip dans _offeredIpAddr
  _sendRequest(true);

  // Reception du paquet DHCP ACK (et enregistrement du reste des adresses
  // et variables) + Gestion du Timeout
  while (!_recvAck()) {
    if (millis() - _beginTime > DHCP_TIMEOUT) return false;
  }

  // Reussite
  return true;
}



//-DHCP--------------------------Release------------------------------------


void DHCPClass::release() {
  // S'occupe du processus de liberation de l'ip et stocke les infos dans les
  // variables publiques

  // On envoi un paquet DHCP Release
  _sendRelease();

  // On remet à zéro l'adresse IP
  memset(ipAddr, 0, 4);
}
