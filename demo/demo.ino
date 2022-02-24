#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "esp_camera.h"
#include <string.h>


//-------------------------------Paramètres------------------------------------

// id de l'ESP
#define ID 99 // 4 digits max

// WiFi network name and password:
const char * networkName = "iPhone de Gaspard";
const char * networkPswd = "gaspardd";

// Adresse IP du serveur UDP :
// either use the ip address of the server or
// a network broadcast address;
const char * udpAddress = "172.20.10.4";
const int udpPort = 44444;

// Est-on déjà connectés ?
boolean connected = false;

// Classe udp pour la biblioteque WifiUDP
WiFiUDP udp;

// Pins CAMERA_MODEL_M5STACK_ESP32CAM
#define PWDN_GPIO_NUM     -1
#define RESET_GPIO_NUM    15
#define XCLK_GPIO_NUM     27
#define SIOD_GPIO_NUM     25
#define SIOC_GPIO_NUM     23

#define Y9_GPIO_NUM       19
#define Y8_GPIO_NUM       36
#define Y7_GPIO_NUM       18
#define Y6_GPIO_NUM       39
#define Y5_GPIO_NUM        5
#define Y4_GPIO_NUM       34
#define Y3_GPIO_NUM       35
#define Y2_GPIO_NUM       17
#define VSYNC_GPIO_NUM    22
#define HREF_GPIO_NUM     26
#define PCLK_GPIO_NUM     21

// Taille max de la donnée transmise par paquet (sortie)
#define DATA_MAX 1024
// Taille max des paquets entrants
#define BUFFER_LEN 64
// Temps avant timeout (en millisecondes)
#define TIMEOUT 5000

//----------------------------------Setup--------------------------------------

void setup(){

  //Desactiver le detecteur de brownout
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  // Initilize hardware serial:
  Serial.begin(115200);
  delay(10);
  Serial.println("Démarrage de l'ESP32cam n°" + ID);

  //Connect to the WiFi network
  connectToWiFi(networkName, networkPswd);

  // Configuration de la caméra
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  /* Format de l'image
     Pour cette carte la qualité max est le CIF (352×288) car pas de PSRAM
     En fait j'ai reussi à aller jusqu'au XGA (1024×768) mais je garanti pas
     qu'il n'y ai aucun problème, la biblioteque dit qu'on ne peut aller
     que jusqu'à CIF                                                         */
  config.frame_size = FRAMESIZE_XGA;
  config.jpeg_quality = 12;  //0-63 (plus petit signifie qualité plus grande)
  //nombre d'image dans le buffer (seulement 1 car pas de PSRAM)
  config.fb_count = 1;

  // Initialisation de la caméra
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.print("L'initialisation de la caméra à ratée : Erreur 0X");
    Serial.println(err, HEX);
    redemmarage();
  }

  Serial.println("Initialisation terminée");
}


//-----------------------------------Run---------------------------------------

void loop(){
  // Ne fonctionne qu'une fois connectée
  if(connected)
  {
    char requete;
    while (Serial.available() != 0)
    {
      requete = Serial.read();
      if (requete == 'p') envoiPhoto();
    }
  }
}

//------------------------------Redemarrage------------------------------------

void redemmarage() {
  //Redemarrage de l'appareil (en cas de problèmes)
  Serial.print("Redémarrage");
  delay(500);
  Serial.print(".");
  delay(500);
  Serial.print(".");
  delay(500);
  Serial.println(".");
  delay(500);
  ESP.restart();
}


//----------------------------------Wifi---------------------------------------

void connectToWiFi(const char * ssid, const char * pwd){
  Serial.println("Connection au réseau Wifi : " + String(ssid));

  // Suppression de l'ancienne configuration
  WiFi.disconnect(true);
  //
  WiFi.onEvent(WiFiEvent);

  //Initialisation de la connection
  // (La fonction fait son retour avant que la connection
  //  soit établie, c'est le gestionnaire d'événement qui
  //  prend la suite)
  WiFi.begin(ssid, pwd);

  Serial.println("Attente d'une connection Wifi...");
}

void WiFiEvent(WiFiEvent_t event){
  //Fonction qui gère les évenements liés au Wifi
    switch(event) {
      case SYSTEM_EVENT_STA_GOT_IP:
          //Quand on se connecte
          Serial.print("WiFi connecté! Adresse IP de la caméra : ");
          Serial.println(WiFi.localIP());
          // Initialisation du client
          // (Ca initialise le buffer)
          udp.begin(WiFi.localIP(),udpPort);
          connected = true;
          Serial.println("Envoyez 'p' pour prendre une photo et l'envoyer au serveur.");
          break;
      case SYSTEM_EVENT_STA_DISCONNECTED:
          // Quand on se deconecte
          Serial.println("Connection Wifi impossible, reconnection...");
          connected = false;
          break;
      default: break;
    }
}


//----------------------Capture et envoi de la photo---------------------------

void envoiPhoto() {
  // Gere la prise d'image et l'envoi de la photo

  // Aquisition de la capture
  camera_fb_t * fb = NULL;
  fb = esp_camera_fb_get();

  // Verification de la capture, redemmarage sinon
  if(!fb) {
    Serial.println("La capture a ratée, l'appareil va redemarrer");
    redemmarage();
  }
  Serial.println("Capture prise !");

  //Variables
  uint8_t *fbBuf = fb->buf; // Pointeur vers le binaire du fichier image
  size_t fbLen = fb->len; // Longueur
  uint8_t buffer[BUFFER_LEN]; // Buffer des paquet entrants

  /*
  // Debug
  for (int i=0; i<fbLen; i++)
    Serial.print(fbBuf[i], HEX);
  Serial.println(); */

  // Determination de la procedure d'envoi
  Serial.println("Taille image : " + String(fbLen) + " octets");
  size_t tailleData = DATA_MAX - 43;
  unsigned int nbPacket = fbLen / tailleData;
  if (fbLen % tailleData > 0) nbPacket++;
  // 43 est la taille de l'entête lors de notre envoi de paquet de data

  // Envoi de la requete jusqu'à réponse positive
  while(!requeteServeur(nbPacket, buffer));

  Serial.println("Envoi en cours de " + String(nbPacket) + " paquets au serveur");

  // On envoie les paquets un à un, chaque paquet est renvoyé
  // en boucle tant que le serveur n'a pas repondu ACK
  for (unsigned int i=0; i<nbPacket; i++)
    while(!envoiPaquet(i, nbPacket, tailleData, buffer, fbBuf, fbLen));

  Serial.println("Envoi terminé");
}


//------------------------Requete d'envoi des données--------------------------

boolean requeteServeur(unsigned int nbPacket, uint8_t buffer[BUFFER_LEN]) {
  /* Envoie une requete d'envoi de photo au serveur
     puis en attend la reponse, elle peut être positive, negative
     ou ne pas avoir lieu (timeout).
     Si la requette obtient un reponse, la fonction revoit true,
     sinon elle revoit false.                                      */

  // Envoi de la requete d'upload
  Serial.println("Envoi de la requete d'upload de l'image au serveur");
  Serial.println(" -> Id de la carte : " + String(ID) + ", Nombre de packets à envoyer : " + String(nbPacket) + ", Taille du buffer : " + String(DATA_MAX));

  // Paquet de demande
  udp.beginPacket(udpAddress,udpPort);
  udp.printf("file upload request, id : %04d, packet : %04d, buffer : %08d", ID, nbPacket, DATA_MAX);
  udp.endPacket();

  // Reponse sensées être reçue du serveur
  char ack[BUFFER_LEN];
  snprintf(ack, BUFFER_LEN, "file request accepted, id : %04d, packet : %04d", ID, nbPacket);
  char nack[BUFFER_LEN];
  snprintf(nack, BUFFER_LEN, "file request refused, id : %04d, packet : %04d", ID, nbPacket);

  // Timer pour timeout
  unsigned long int timer = millis();

  while (true)
  {
    // Remise à zéro du buffer et mise en place de la reception
    memset(buffer, 0, BUFFER_LEN);
    udp.parsePacket();

    // Lecture des paquets receptionnés
    if (udp.read(buffer, BUFFER_LEN) > 0)
    {
      if (!strcmp((char *)buffer,ack)) // Reponse correspondant à une reponse positive
      {
        Serial.println(" <- Reponse positive du serveur, démarrage de l'envoi...");
        return true;
      }
      else if (!strcmp((char *)buffer,nack)) // Reponse correspondant à une reponse negative
      {
        Serial.println(" <- Reponse negative du serveur, tentative de réenvoi de la requete...");
        delay(1000);
        return false;
      }

    }
    // Delai de reponse expiré
    if (millis() > timer+TIMEOUT)
    {
      Serial.println(" >< Timeout, tentative de réenvoi de la requete...");
      delay(1000);
      return false;
    }
  }
}


//-----------------------------Envoi d'un paquet-------------------------------

boolean envoiPaquet(unsigned int idP, unsigned int nbPacket, size_t tailleData, uint8_t buffer[BUFFER_LEN], uint8_t *fbBuf, size_t fbLen) {
  /* Envoie le packet numero idP au serveur puis attend la
     confirmation de bonne reception, elle peut être positive, negative
     ou ne pas avoir lieu (timeout).
     Si la requette obtient un reponse, la fonction revoit true,
     sinon elle revoit false.                                      */

  Serial.println(" -> Envoi paquet " + String(idP+1) + "/" + String(nbPacket));

  // On se met à l'emplacement du buffer correspondant au paquet à envoyer
  fbBuf += idP * tailleData;

  // Envoi du paquet avec les données
  if (((idP+1)*tailleData) < fbLen) { // Envoi des premiers paquets
    udp.beginPacket(udpAddress,udpPort);
    udp.printf("file upload, id : %04d, idP : %04d, data : ", ID, idP);
    udp.write(fbBuf,tailleData);
    udp.endPacket();
  }
  else if (fbLen%tailleData > 0) { // Envoi du dernier paquet (car plus petit que les autres)
    size_t remainder = fbLen%tailleData;
    udp.beginPacket(udpAddress,udpPort);
    udp.printf("file upload, id : %04d, idP : %04d, data : ", ID, idP);
    udp.write(fbBuf,remainder);
    udp.endPacket();
  }

  // Reponse sensées être reçue du serveur
  char ack[BUFFER_LEN];
  snprintf(ack, BUFFER_LEN, "ACK, id : %04d, idP : %04d", ID, idP);
  char nack[BUFFER_LEN];
  snprintf(nack, BUFFER_LEN, "NACK, id : %04d, idP : %04d", ID, idP);

  // Timer pour timeout
  unsigned long int timer = millis();

  //Reception de la confirmation
  while (true)
  {
    // Remise à zéro du buffer et mise en place de la reception
    memset(buffer, 0, BUFFER_LEN);
    udp.parsePacket();

    // Lecture des paquets receptionnés
    if (udp.read(buffer, BUFFER_LEN) > 0)
    {
      if (!strcmp((char *)buffer,ack)) // Reponse correspondant à une bonne reception
      {
        Serial.println(" <- ACK paquet n°" + String(idP+1));
        return true;
      }
      else if (!strcmp((char *)buffer,nack)) // Reponse correspondant à une mauvaise reception
      {
        Serial.println(" <- NACK paquet n°" + String(idP+1));
        delay(1000);
        return false;
      }

    }
    // Delai de reponse expiré
    if (millis() > timer+TIMEOUT)
    {
      Serial.println(" >< Timeout, tentative de réenvoi du paquet...");
      delay(1000);
      return false;
    }
  }
}
