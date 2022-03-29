/******************************************************************************
 * Pour utiliser un shield Ethernet équipé d'une w5500.                       *
 * On utilise le HSPI et les pin 15 pour le Chip Select ainsi que le 32 pour  *
 * l'Interrupt (Cette bibliothèque w5500HSPI.h pourrait être adaptée à plus   *
 * de cas en rajoutant deux trois trucs mais ici ça ne nous interesse pas.)   *
 * Pour voir la doc de la w5500 :                                             *
 * - https://docs.wiznet.io/Product/iEthernet/W5500/overview                  *
 * - https://docs.wiznet.io/Product/iEthernet/W5500/datasheet                 *
 ******************************************************************************/

#ifndef W5500_HSPI_H
#define W5500_HSPI_H

#include <Arduino.h>
#include <SPI.h>

//-----------------------------------spi---------------------------------------

// Pins (on ne peut modifier que le PIN_CS et le PIN_INT)
#define HSPI_MISO   12
#define HSPI_MOSI   13
#define HSPI_SCLK   14
#define PIN_CS      15
#define PIN_INT     32

#define SPI_ETHERNET_SETTINGS SPISettings(26600000, MSBFIRST, SPI_MODE0)
//#define SPI_ETHERNET_SETTINGS SPISettings(80000000, MSBFIRST, SPI_MODE0)
// Normalement, en utilisant le HSPI, on a pas besoin d'utiliser la GPIO Matrix
// et pourrait donc théoriquement monter la fréquence a 80 MHz
// Le problème c'est qu'on y arrive pas et on sait pas pourquoi, on se retrouve
// à être limité à 26,6 MHz comme si on utilisait la GPIO Matrix

static SPIClass * hspi = NULL;


//-----------------------------fonctions-w5500---------------------------------

// La classe pour les fonctions liés à la w5500
class W5500Class {

  public:

    // Etat du pin INT
    static inline boolean stateINT() {return !digitalRead(PIN_INT);}

    // Initialisation
    boolean init();
    boolean softReset();

    // Write
    void writeByte(uint8_t bsb, uint16_t offsetAddr, uint8_t value); // Ecrit un octet ou char
    void write2Bytes(uint8_t bsb, uint16_t offsetAddr, const uint8_t * valueBuffer); // Ecrit deux octet en tableau[2]
    void writeShort(uint8_t bsb, uint16_t offsetAddr, uint16_t value); // Ecrit un short
    void write4Bytes(uint8_t bsb, uint16_t offsetAddr, const uint8_t * valueBuffer); // Ecrit quatre octet en tableau[4]
    void writeInt(uint8_t bsb, uint16_t offsetAddr, uint32_t value); // Ecrit un int
    void writeLen(uint8_t bsb, uint16_t offsetAddr, const uint8_t * valueBuffer, size_t len); // Ecrit (len) octet en tableau[len]

    // Read
    uint8_t readByte(uint8_t bsb, uint16_t offsetAddr);
    void read2Bytes(uint8_t bsb, uint16_t offsetAddr, uint8_t * valueBuffer); // Ecrit deux octet en tableau[2]
    uint16_t readShort(uint8_t bsb, uint16_t offsetAddr);
    void read4Bytes(uint8_t bsb, uint16_t offsetAddr, uint8_t * valueBuffer); // Ecrit deux octet en tableau[2]
    uint32_t readInt(uint8_t bsb, uint16_t offsetAddr);
    void readLen(uint8_t bsb, uint16_t offsetAddr, uint8_t * valueBuffer, size_t len);
    // /!\ Si on utilise un buffer, il faut que le buffer ai dejà la mémoire
    //     necessaire d'allouée pour la donnée (içi, 2, 4 ou len)

    // Plus d'infos dans le cpp


    // BSB (Block Select Bits) : adresse pour la selection des blocs dans la
    // phase de controle de la communcation SPI (voir doc w5500)
    static const uint8_t commonRegister = 0b00000;
    static const uint8_t socket0Register = 0b00001;
    static const uint8_t socket0TXBuffer = 0b00010;
    static const uint8_t socket0RXBuffer = 0b00011;
    static const uint8_t socket1Register = 0b00101;
    static const uint8_t socket1TXBuffer = 0b00110;
    static const uint8_t socket1RXBuffer = 0b00111;
    // ...
    // On ne defini pas les autres car on en a pas besoin

  private :

    // Fonction pour la communication SPI avec la w5500
    static inline void initCS() {pinMode(PIN_CS, OUTPUT);}
    static inline void setCS() {digitalWrite(PIN_CS, LOW);}
    static inline void resetCS() {digitalWrite(PIN_CS, HIGH);}
    static inline void initINT() {pinMode(PIN_INT, INPUT);}

};

extern W5500Class w5500;
// On peut se permettre d'initialiser la classe içi car il y en aura qu'une


//---------------------------------Autres--------------------------------------

#endif

#ifndef UTIL_H
#define UTIL_H

// Fonctions htons, ntohs, htonl et ntohl pour communiquer des valeurs de
// plusieurs octet sur le réseau quel que soit le boutisme
#define htons(x) ( (((x)<<8)&0xFF00) | (((x)>>8)&0xFF) )
#define ntohs(x) htons(x)

#define htonl(x) ( ((x)<<24 & 0xFF000000UL) | \
                   ((x)<< 8 & 0x00FF0000UL) | \
                   ((x)>> 8 & 0x0000FF00UL) | \
                   ((x)>>24 & 0x000000FFUL) )
#define ntohl(x) htonl(x)

#endif
