/******************************************************************************
 * Pour utiliser un shield Ethernet équipé d'une w5500.                       *
 * On utilise le HSPI et les pin 15 pour le Chip Select ainsi que le 32 pour  *
 * l'Interrupt (Cette bibliothèque w5500HSPI.h pourrait être adaptée à plus   *
 * de cas en rajoutant deux trois trucs mais ici ça ne nous interesse pas.)   *
 * Pour voir la doc de la w5500 :                                             *
 * - https://docs.wiznet.io/Product/iEthernet/W5500/overview                  *
 * - https://docs.wiznet.io/Product/iEthernet/W5500/datasheet                 *
 ******************************************************************************/

#include "w5500HSPI.h"

//---------------------------------w5500Class---------------------------------

//-w5500---------------------------init/reset----------------------------------


boolean W5500Class::init() {
  // HSPI
  hspi = new SPIClass(HSPI);
  w5500.initCS();
  initINT();
  resetCS();
  hspi->begin();

  // Reglages du w5500 (voir doc w5500)
  if (!softReset()) return false; // Si le reset à pas fonctioné
  writeByte(commonRegister, 0x0016, 0b11000000); // Activation de l'interrupt pour les conflits d'IP et destination inaccessibles
  writeByte(commonRegister, 0x0018, 0b00000001); // Activation de l'interrupt pour le socket 0 seul (car c'est le seul qu'on utilise)

  return true; // Init réussi
}


boolean W5500Class::softReset() {
  unsigned char count = 0;
  writeByte(commonRegister, 0x0000, 0b10000000); // Declenchement du soft-reset dans le MR
  do {
    if (readByte(commonRegister, 0x0000)==0) return true; // On test si c'est réussi
    delay(2);
  } while (count++ < 20);
  return false; // Raté
}


//-w5500-----------------------------write-------------------------------------


void W5500Class::writeByte(uint8_t bsb, uint16_t offsetAddr, uint8_t value) {
  // Demande l'écriture et écrit l'octet correspondant au bloc bsb et à
  // l'adresse offsetAddr

  // Creation de la communication
  uint8_t com[4];
  com[0] = offsetAddr >> 8; // Adress Phase 1 (voir doc w5500)
  com[1] = offsetAddr & 0xFF; // Adress Phase 2 (voir doc w5500)
  com[2] = (bsb << 3) | 0b101; // Control Phase (voir doc w5500)
  com[3] = value; // Data Phase

  // Transfert de la communication
  hspi->beginTransaction(SPI_ETHERNET_SETTINGS);
  setCS();
  hspi->transfer(com, 4); // Envoi
  resetCS();
  hspi->endTransaction();
}


void W5500Class::write2Bytes(uint8_t bsb, uint16_t offsetAddr, const uint8_t * valueBuffer) {
  // Demande l'écriture et écrit les deux octets correspondant au bloc bsb et à
  // l'adresse offsetAddr

  // Creation de la communication
  uint8_t com[5];
  com[0] = offsetAddr >> 8; // Adress Phase 1 (voir doc w5500)
  com[1] = offsetAddr & 0xFF; // Adress Phase 2 (voir doc w5500)
  com[2] = (bsb << 3) | 0b110; // Control Phase (voir doc w5500)
  memcpy(com+3, valueBuffer, 2); // Data Phase (voir doc w5500)

  // Transfert de la communication
  hspi->beginTransaction(SPI_ETHERNET_SETTINGS);
  setCS();
  hspi->transfer(com, 5); // Envoi de la commande et data
  resetCS();
  hspi->endTransaction();
}


void W5500Class::writeShort(uint8_t bsb, uint16_t offsetAddr, uint16_t value) {
  // Demande l'écriture et écrit les deux octets correspondant au bloc bsb et
  // à l'adresse offsetAddr (prend un short en argument)

  // Convertion de value en buffer
  uint8_t valueBuffer[2];
  value = htons(value);
  memcpy(valueBuffer, &value, sizeof(uint32_t));

  // Demande et ecriture
  write2Bytes(bsb, offsetAddr, valueBuffer);
}


void W5500Class::write4Bytes(uint8_t bsb, uint16_t offsetAddr, const uint8_t * valueBuffer) {
  // Demande l'écriture et écrit les quatre octets correspondant au bloc bsb et
  // à l'adresse offsetAddr

  // Creation de la communication
  uint8_t com[7];
  com[0] = offsetAddr >> 8; // Adress Phase 1 (voir doc w5500)
  com[1] = offsetAddr & 0xFF; // Adress Phase 2 (voir doc w5500)
  com[2] = (bsb << 3) | 0b111; // Control Phase (voir doc w5500)
  memcpy(com+3, valueBuffer, 4); // Data Phase (voir doc w5500)

  // Transfert de la communication
  hspi->beginTransaction(SPI_ETHERNET_SETTINGS);
  setCS();
  hspi->transfer(com, 7); // Envoi de la commande et de la data
  resetCS();
  hspi->endTransaction();
}


void W5500Class::writeInt(uint8_t bsb, uint16_t offsetAddr, uint32_t value) {
  // Demande l'écriture et écrit les quatre octets correspondant au bloc bsb et
  // à l'adresse offsetAddr (prend un int en argument)

  // Convertion de value en buffer
  uint8_t valueBuffer[4];
  value = htonl(value);
  memcpy(valueBuffer, &value, sizeof(uint32_t));

  // Demande et ecriture
  write4Bytes(bsb, offsetAddr, valueBuffer);
}


void W5500Class::writeLen(uint8_t bsb, uint16_t offsetAddr, const uint8_t * valueBuffer, size_t len) {
  // Demande l'écriture et écrit les (len) octets correspondant au bloc bsb et à
  // l'adresse offsetAddr

  // Creation de la communication
  uint8_t * com = (uint8_t *)malloc(len+3); // Allocation dynamique d'un espace mémoire (un uint8_t fait 1 octet)
  if (com == NULL) return ;
  com[0] = offsetAddr >> 8; // Adress Phase 1 (voir doc w5500)
  com[1] = offsetAddr & 0xFF; // Adress Phase 2 (voir doc w5500)
  com[2] = (bsb << 3) | 0b100; // Control Phase (voir doc w5500)
  memcpy(com+3, valueBuffer, len); // Data Phase (voir doc w5500)

  // Transfert de la communication
  hspi->beginTransaction(SPI_ETHERNET_SETTINGS);
  setCS();
  hspi->transfer(com, len + 3); // Envoi de la commande et de la data
  resetCS();
  hspi->endTransaction();

  free(com); // Liberation de l'espace mémoire
}



//-w5500------------------------------read-------------------------------------


uint8_t W5500Class::readByte(uint8_t bsb, uint16_t offsetAddr) {
  // Demande la lecture et lit l'octet correspondant au bloc bsb et à
  // l'adresse offsetAddr puis le retourne

  // Creation de la commande
  uint8_t cmd[3];
  cmd[0] = offsetAddr >> 8; // Adress Phase 1 (voir doc w5500)
  cmd[1] = offsetAddr & 0xFF; // Adress Phase 2 (voir doc w5500)
  cmd[2] = (bsb << 3) | 0b001; // Control Phase (voir doc w5500)

  // Transfert de la commande et reception de la donnée
  hspi->beginTransaction(SPI_ETHERNET_SETTINGS);
  setCS();
  hspi->transfer(cmd, 3); // Envoi de le commande
  uint8_t value = hspi->transfer(0); // Reception et enregistrement
  resetCS();
  hspi->endTransaction();

  return value;
}


void W5500Class::read2Bytes(uint8_t bsb, uint16_t offsetAddr, uint8_t * valueBuffer) {
  // Demande la lecture et lit les deux octet correspondant au bloc bsb et à
  // l'adresse offsetAddr la stocke dans valueBuffer

  // Creation de la commande
  uint8_t cmd[3];
  cmd[0] = offsetAddr >> 8; // Adress Phase 1 (voir doc w5500)
  cmd[1] = offsetAddr & 0xFF; // Adress Phase 2 (voir doc w5500)
  cmd[2] = (bsb << 3) | 0b010; // Control Phase (voir doc w5500)

  // Transfert de la commande et reception de la donnée
  hspi->beginTransaction(SPI_ETHERNET_SETTINGS);
  setCS();
  hspi->transfer(cmd, 3); // Envoi de le commande
  hspi->transfer(valueBuffer, 2); // Reception et enregistrement
  resetCS();
  hspi->endTransaction();
}


uint16_t W5500Class::readShort(uint8_t bsb, uint16_t offsetAddr) {
  // Demande la lecture et lit les deux octet correspondant au bloc bsb et à
  // l'adresse offsetAddr la retourne sous forme de short

  // Stockage de la donnée
  uint8_t valueBuffer[2];

  // Demande et reception des données
  read2Bytes(bsb, offsetAddr, valueBuffer);

  // Convertir en uint16_t puis retour
  uint16_t value;
  memcpy(&value, valueBuffer, sizeof(uint16_t));
  value = ntohs(value);
  return value;
}


void W5500Class::read4Bytes(uint8_t bsb, uint16_t offsetAddr, uint8_t * valueBuffer) {
  // Demande la lecture et lit les quatre octet correspondant au bloc bsb et à
  // l'adresse offsetAddr puis la stocke dans valueBuffer

  // Creation de la commande
  uint8_t cmd[3];
  cmd[0] = offsetAddr >> 8; // Adress Phase 1 (voir doc w5500)
  cmd[1] = offsetAddr & 0xFF; // Adress Phase 2 (voir doc w5500)
  cmd[2] = (bsb << 3) | 0b011; // Control Phase (voir doc w5500)

  // Transfert de la commande et reception de la donnée
  hspi->beginTransaction(SPI_ETHERNET_SETTINGS);
  setCS();
  hspi->transfer(cmd, 3); // Envoi de le commande
  hspi->transfer(valueBuffer, 4); // Reception et enregistrement
  resetCS();
  hspi->endTransaction();
}


uint32_t W5500Class::readInt(uint8_t bsb, uint16_t offsetAddr) {
  // Demande la lecture et lit les quatre octet correspondant au bloc bsb et à
  // l'adresse offsetAddr puis la retourne sous forme de int

  // Stockage de la donnée
  uint8_t valueBuffer[4];

  read4Bytes(bsb, offsetAddr, valueBuffer);

  // Convertir en uint16_t puis retour
  uint32_t value;
  memcpy(&value, valueBuffer, sizeof(uint32_t));
  value = ntohl(value);
  return value;
}


void W5500Class::readLen(uint8_t bsb, uint16_t offsetAddr, uint8_t * valueBuffer, size_t len) {
  // Demande la lecture et lit les (len) octet correspondant au bloc bsb et à
  // l'adresse offsetAddr puis les mets dans le buffer
  // /!\ Il faut que le buffer ai dejà la mémoire necessaire d'allouée

  // Creation de la commande
  uint8_t cmd[3];
  cmd[0] = offsetAddr >> 8; // Adress Phase 1 (voir doc w5500)
  cmd[1] = offsetAddr & 0xFF; // Adress Phase 2 (voir doc w5500)
  cmd[2] = (bsb << 3) | 0b000; // Control Phase (voir doc w5500)

  // Transfert de la commande et reception de la donnée
  hspi->beginTransaction(SPI_ETHERNET_SETTINGS);
  setCS();
  hspi->transfer(cmd, 3); // Envoi de le commande
  hspi->transfer(valueBuffer, len); // Reception et enregistrement
  resetCS();
  hspi->endTransaction();
}



//-w5500--------------------------declaration----------------------------------

W5500Class w5500;
