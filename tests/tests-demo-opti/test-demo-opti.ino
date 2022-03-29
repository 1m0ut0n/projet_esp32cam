#include <SPI.h>
#include "w5500HSPI.h"
#include "EthernetUDP.h"

const byte mac[6] = {0x00, 0xAA, 0xBB, 0xCC, 0xDE, 0x02};

const unsigned short udpPort = 44444;
const unsigned short portServeur = 44444;
const byte ipServeur[4] = {192, 168, 10, 86};

//#define DEBUG

uint8_t * textEnvoi;
uint16_t tailleRecue=0;


void setup() {

  Serial.begin(115200);
  while(!Serial);

  Ethernet.begin(mac);
  udp.begin(udpPort);
  udp.configServer(portServeur, ipServeur);

  Serial.println("Initialisation terminée");
}

void loop() {

  tailleRecue = w5500.readShort(w5500.socket0Register, 0x0026);
  uint8_t interruptStatus = w5500.stateINT();
  uint16_t taille = udp.parsePacket();

  #ifdef DEBUG
  Serial.print("Remplissage Buffer RX : " + String(tailleRecue, DEC) + "   Etat Interrupt : ");
  if (interruptStatus) Serial.print("INT");
  else Serial.print("NINT");
  Serial.println("   Taille Prochain Paquet : " + String(taille));
  #endif


  if (taille) {

    Serial.println("Reception :");

    uint8_t * buffer;
    buffer = (uint8_t *) malloc(taille);
    if (buffer) {

      udp.read(buffer, taille);

      Serial.print("--> ");
      for (uint16_t i=0; i<taille; i++)
        Serial.print((char)buffer[i]);
      Serial.println();

      Serial.println("Reception réussie");
    }
  }

  uint16_t caracteresDispo = Serial.available();
  if (caracteresDispo) {

    Serial.println("Envoi :");

    textEnvoi = (uint8_t *) malloc(caracteresDispo);

    Serial.print("<-- ");

    for (uint16_t i=0; i<caracteresDispo; i++) {
      char caractere = Serial.read();
      if (caractere == '\n') caractere = ' ';
      textEnvoi[i] = caractere;
      Serial.print((char) caractere);
    }
    Serial.println();

    boolean reussite = true;
    if (!udp.beginPacket()) reussite = false;
    if (!udp.write(textEnvoi, caracteresDispo)) reussite = false;
    if (!udp.endPacket()) reussite = false;

    if (reussite) Serial.println("Envoi réussi");
    else Serial.println("Envoi raté");

    free(textEnvoi);

  }
  delay(2000);
}
