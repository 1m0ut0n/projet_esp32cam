#include <SPI.h>

#define HSPI_MISO   12
#define HSPI_MOSI   13
#define HSPI_SCLK   14
#define PIN_CS     15

#define SPI_ETHERNET_SETTINGS SPISettings(26600000, MSBFIRST, SPI_MODE0)

//uninitalised pointers to SPI objects
SPIClass * hspi = NULL;

void setup() {
  //initialise HSPI SPIClass
  hspi = new SPIClass(HSPI);

  Serial.begin(115200);

  //initialise hspi with default pins
  //SCLK = 14, MISO = 12, MOSI = 13, SS = 15
  hspi->begin();

  //set up chip select pins as outputs as the Arduino API
  //doesn't handle automatically pulling CS low
  pinMode(PIN_CS, OUTPUT); //HSPI CS

}

void loop() {
  Serial.println("Chip Version : " + String(readChipVersion()));
  delay(5000);
}

void setCS() {digitalWrite(PIN_CS, LOW);}
void resetCS() {digitalWrite(PIN_CS, HIGH);}

int readChipVersion(){
  uint8_t cmd[3];
  cmd[0] = 0;
  cmd[1] = 0x39;
  cmd[2] = 0b00000001;

  hspi->beginTransaction(SPI_ETHERNET_SETTINGS);
  setCS();
  hspi->transfer(cmd, 3);
  unsigned int chipVersion = hspi->transfer(0);
  resetCS();
  hspi->endTransaction();

  return chipVersion;
}
