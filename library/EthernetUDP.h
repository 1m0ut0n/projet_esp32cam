/******************************************************************************
 * Pour utiliser un shield Ethernet équipé d'une w5500.                       *
 * On utilise le HSPI et on ne configure que 2 socket (pour avoir 8k de ram)  *
 * en mode UDP et on configure un serveur unique avec lequel se fera la       *
 * communication. (Tout autre paquet sera ignoré.)                            *
 * Cette bibliothèque EthernetUDP.h est optimisée pour notre utilisation      *
 * specifique.                                                                *
 * Pour voir la doc de la w5500 :                                             *
 * - https://docs.wiznet.io/Product/iEthernet/W5500/overview                  *
 * - https://docs.wiznet.io/Product/iEthernet/W5500/datasheet                 *
 ******************************************************************************/

#ifndef ETHERNET_UDP_H
#define ETHERNET_UDP_H

#include <Arduino.h>
#include <SPI.h>
#include "w5500HSPI.h"

//--------------------------------Options--------------------------------------

// Pour filtrer seulement les paquet provenant du serveur (avec le port ou non)
#define VERIFY_PROVENANCE
//#define VERIFY_PORT_PROVENANCE

// Max qu'on puisse avoir par paquet
#define MTU 1460

//--------------------------------Ethernet-------------------------------------

enum ethernetIssue {
  NO_PROBLEM,
  IP_CONFLICT,
  DESTINATION_UNREACHABLE
};

enum ethernetHardwareStatus {
  EthernetNoHardware,
  EthernetHasHardware
};

enum ethernetLinkStatus {
  LinkOFF,
  LinkON
};

// La classe pour les fonctions Ethernet
class EthernetClass {

  public :

    // Configurations
    inline void init() {w5500.init();};
    // Initialise la communication SPI avec le shield et fait les première
    // configuration
    boolean begin(const uint8_t * mac, const uint8_t * gateway, const uint8_t * subnet, const uint8_t * ip);
    // l'adresse mac est un tableu de 6 uint8_t, les adresses ip sont des tableau de 4 uint8_t
    boolean begin(const uint8_t * mac, const uint8_t * ip);
    // Met un gateway et un subnet par défaut (x.x.x.1 et 255.255.255.0)

    // DHCP
    boolean begin(const uint8_t * mac);
    // Configuration et adressage en DHCP
    uint8_t maintain();
    // Renouvelle le bail si besoin

    // Status
    ethernetHardwareStatus hardwareStatus();
    // Return EthernetNoHardware ou EthernetHasHardware suivant si le shield
    // est correctement connecté/alimenté ou non
    // /!\ return EthernetNoHardware si c'est le mauvais shield
    ethernetLinkStatus linkStatus();
    // Return LinkON ou LinkOFF suivant si le cable est branché ou non
    String localIP();
    // Return un String de l'adresse IP de l'appareil
    ethernetIssue checkProblem(boolean ok = true);
    // Verifie les problèmes de conflit d'IP ou de destination
    // ok sert a savoir si on a pris connaissance ou non du problème et donc
    // de reset l'interrupt si oui

  // Plus d'info dans le cpp
};



extern EthernetClass Ethernet;
// On peut se permettre d'initialiser la classe içi car il y en aura qu'une




//--------------------------------UDP-------------------------------------

// Classe pour les fonctions udpPort
// /!\ Specifique à notre bibliothèque (on ne configure qu'un socket avec 16k
//     octets de ram et on parle à un serveur unique)
class UDPClass {

  public :

    // Constructeur de la classe avec le socket utilisé (dispo 0 et 1)
    // La 1 est deja utilisée par le DHCP
    UDPClass(); // Cherche le premier socket (fonctionne pas jsp pk)
    UDPClass(uint8_t socketN);

    // Destructeur de la classe (appel la fonction close avant sa destruction)
    ~UDPClass();

    // Fonctions d'initialisation
    boolean begin(unsigned short port);
    // On configure un unique socket sur la w5500 en UDP
    void configServer(unsigned short portDest, const uint8_t * ipDest);
    // On configure à l'avance notre destination et port car on communiquera avec
    // un serveur unique


    // Fonctions de lecture
    uint16_t parsePacket();
    // Si il y'a un paquet, retourne la taille de celui-ci, retourne 0 sinon
    // /!\ Si il y'a un paquet mais venant d'un autre serveur que celui
    //     enregistré avec configServer, le parsePacket retournera 0
    // /!\ Ne process que le prochain paquet et abandonne celui d'avant si il
    //     n'as pas entierement été lu
    uint8_t read();
    uint16_t read(uint8_t * buffer, uint16_t len);
    // Lit les prochains caractères du paquet actuel
    // /!\ A appeler après un parsePacket
    // /!\ Si on va trop loin, il retourne 0 ou ne touche pas au reste du buffer
    // /!\ Ne verifie pas si le buffer est assez grand

    // Fonctions d'écriture
    // Elles retournent la reussite ou non de l'ecriture/envoi
    boolean beginPacket();
    // Initialise en remettant à 0 le buffer
    boolean write(uint8_t octet);
    // Ecrire un octet dans le buffer TX
    boolean write(const uint8_t* buffer, uint16_t len);
    // Ecrire len octet (max MTU) dans le buffer TX
    // /!\ On doit avoir len<= taille du buffer
    boolean endPacket();
    // Fait la demande d'envoi au serveur indiqué dans configServeur et
    // retourne sa reussite ou non

    // Fonction de fin
    void close();
    // Fermeture du socket


  private :

    //Registers
    uint8_t _socket = 8; // Valeur pas de socket
    uint8_t _socketRegister = 0b01;
    uint8_t _TXRegister = 0b10;
    uint8_t _RXRegister = 0b11;

    // Octets restants à lire
    uint16_t _octetsRestant=0;
    uint16_t _octetsEcrits=0;

    // Info sur la communication
    uint16_t _udpPort; // Port pour la communication UDP
    uint16_t _portServeur; // Port de destination
    uint8_t _ipServeur[4]; // Ip de destination

  // Plus d'info dans le cpp
};


#endif
