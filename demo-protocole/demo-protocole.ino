/******************************************************************************
 * TODO :                                                                     *
 *  - Certaines variables pourraient être misent en variables globales plutôt *
 *    qu'en argument (même des const)                                         *
 *  - Certaines variables pourraient prendre moins d'espace mémoire           *
 ******************************************************************************/

#include <Arduino.h>
#include <SPI.h>
#include "w5500HSPI.h" // Bibliothèque custom pour la communication avec une w5500 avec le HSPI
#include "EthernetUdp.h" // Bibliothèque custom pour l'Ethernet en UDP pour cette application
#include "soc/soc.h" // Pour le brownout
#include "soc/rtc_cntl_reg.h" // Pour le brownout
#include "esp_camera.h"
#include <string.h>



//-------------------------------Paramètres------------------------------------

// Type de Client : Photographie des câbles (voir protocole)
#define CLIENT_TYPE 1
// id de l'ESP
#define ID 99 // 4 digits max

// Adresse mac de l'esp32
byte mac[] = {
  0x00, 0xAA, 0xBB, 0xCC, 0xDE, 0x02
};

// Adresse IP  et port du serveur et port pour la communication UDP :
const uint8_t serverAddress[4] = {192,168,10,86};
const uint16_t serverPort = 44444;
const uint16_t udpPort = 44444;

// Taille max de la donnée transmise par paquet (sortie)
#define DATA_MAX 1460 // Max 1460 (MTU)
// Taille max des paquets entrants
#define BUFFER_LEN 32
// Temps avant timeout (en millisecondes)
#define TIMEOUT 1000
// Nombre de fois avant abandon d'un l'envoi
#define ABANDON 5
// Temps à attendre en cas de reponse négative ou timeout avant le réenvoi
// d'un paquet
#define WAIT_TIME 100
// Taille de l'entete lors d'un envoi de données (en octets)
#define ENTETE_LEN 8
// Pin Chip Select de l'Ethernet
#define PIN_CS 15

// Pins CAMERA_MODEL_WROVER_KIT
#define PWDN_GPIO_NUM    -1
#define RESET_GPIO_NUM   -1
#define XCLK_GPIO_NUM    21
#define SIOD_GPIO_NUM    26
#define SIOC_GPIO_NUM    27

#define Y9_GPIO_NUM      35
#define Y8_GPIO_NUM      34
#define Y7_GPIO_NUM      39
#define Y6_GPIO_NUM      36
#define Y5_GPIO_NUM      19
#define Y4_GPIO_NUM      18
#define Y3_GPIO_NUM       5
#define Y2_GPIO_NUM       4
#define VSYNC_GPIO_NUM   25
#define HREF_GPIO_NUM    23
#define PCLK_GPIO_NUM    22

//LEDS
#define LED 2

// Pour avoir des info envoyées à l'ordinateur par le port serial (décommenter)
#define INFO
// Pour avoir plus d'info envoyées à l'ordinateur par le port serial
// (décommenter)
// /!\ Augmente considérablement les temps d'envoi
//#define MORE_INFO



//----------------------------------Setup--------------------------------------

// Variables globales :
// Buffer de reception
uint8_t bufferReception[BUFFER_LEN];
// Taille de la donnée dans un paquet
const size_t tailleData = DATA_MAX - ENTETE_LEN;
// Classe UDP
UDPClass udp; // Sur le socket premier socket libre

// Setup
void setup(){

  //Desactiver le detecteur de brownout
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  // Initialisation de la connection Serial:
  Serial.begin(115200);
  while (!Serial);
  #ifdef INFO
  Serial.println();
  Serial.println("  ________________________________________________________________");
  Serial.println(" /  _____           _         _            _____                  \\");
  Serial.println(" | |     |___ ___ _| |___ ___|_|___ ___   |   __|___ ___ _ _ ___  |");
  Serial.println(" |  |   ||   | . | . | -_|_ -| | . |   |  |  |  |  _| . | | | . | |");
  Serial.println(" | |_____|_|_|___|___|___|___|_|_  |_|_|  |_____|_| |___|___|  _| |");
  Serial.println(" |                             |___|                        |_|   |");
  Serial.println(" \\________________________________________________________________/");
  Serial.println();
  Serial.println("Démarrage de l'ESP32cam n°" + String(ID));
  #endif

  // LEDs
  pinMode(LED, OUTPUT);

  //Connect to the WiFi network
  ethernetConnection();

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

  // Format de l'image UXGA (1600x1200) (le meilleur)
  config.frame_size = FRAMESIZE_UXGA;
  config.jpeg_quality = 10;  //0-63 (plus petit signifie qualité plus grande)
  //nombre d'image dans le buffer (on peut avoir 2 car on a la PSRAM)
  config.fb_count = 2;

  // Initialisation de la caméra
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    #ifdef INFO
    Serial.print("L'initialisation de la caméra à ratée : Erreur 0x");
    Serial.println(err, HEX);
    #endif
    redemmarage();
  }

  #ifdef INFO
  Serial.println("Initialisation terminée");
  Serial.println("Envoyez 'p' pour prendre une photo et l'envoyer au serveur.");
  #endif
}



//-----------------------------------Run---------------------------------------

void loop(){

  if(bailDHCP()) // Ne fonctionne qu'une fois connectée
  {
    char requete;
    while (Serial.available()) // Si on a des données en entrée dans le port Serial
    {
      requete = Serial.read(); // On les lits
      if (requete == 'p') { // Si il y a le charactere p on declenche l'envoi d'une photo

        if (envoiPhoto()) { // Si l'envoi a reussi
          #ifdef INFO
          Serial.println("Envoyez 'p' pour prendre une photo et l'envoyer au serveur.");
          #endif
        }
        else { // Si l'envoi a raté
          #ifdef INFO
          Serial.println("Vérifiez que le serveur est allumé et connecté à la bonne adresse IP. Si le problème persiste, redémarrez l'esp et le serveur.");
          #endif
        }
      }
    }
  }
}



//------------------------------Redemarrage------------------------------------

void redemmarage() {
  //Redemarrage de l'appareil (en cas de problèmes)
  #ifdef INFO
  Serial.print("Redémarrage");
  #endif
  delay(500);
  #ifdef INFO
  Serial.print(".");
  #endif
  digitalWrite(LED,HIGH);
  delay(500);
  #ifdef INFO
  Serial.print(".");
  #endif
  digitalWrite(LED,LOW);
  delay(500);
  #ifdef INFO
  Serial.println(".");
  #endif
  digitalWrite(LED,HIGH);
  delay(500);
  digitalWrite(LED,LOW);
  //Serial.end() le restart ne fonctionne plus après ça
  ESP.restart();
}



//--------------------------------Ethernet-------------------------------------

void ethernetConnection(){
  // Initialise la connection Ethernet

  // Configuration du pin CS pour l'Ethernet
  Ethernet.init();

  // Initialisation de la connection Ethernet
  #ifdef INFO
  Serial.print("Initialisation de la connection Ethernet avec DHCP");
  #endif
  if (!Ethernet.begin(mac)) {
    #ifdef INFO
    Serial.println();
    Serial.print("Echec de la configuration de l'Ethernet avec DHCP : ");
    #endif
    if (Ethernet.hardwareStatus() == EthernetNoHardware ) {
      #ifdef INFO
      Serial.println("Le shield Ethernet n'a pas été trouvé.");
      #endif

    }
    else if (Ethernet.linkStatus() == LinkOFF) {
      #ifdef INFO
      Serial.println("Le câble Ethernet n'est pas connecté.");
      #endif
    }
    // Comme ça n'as pas fonctionné, on redemmare l'ESP
    #ifdef INFO
    Serial.println("L'ESP va redemmarer.");
    #endif
    redemmarage();
  }

  // Si on est connectés
  #ifdef INFO
  Serial.println(" : réussie");
  Serial.print("Adresse IP : ");
  Serial.println(Ethernet.localIP());
  #endif
  udp.begin(udpPort);
  udp.configServer(serverPort, serverAddress);
}

int bailDHCP() {
  // Fonction qui verifie regulièrement que le bail DHCP est toujours valide
  // et qui demande son renouvelement ou un nouveau dans le cas contraire
  // (Un bail est donné pour un certain temps)

  switch (Ethernet.maintain()) {
    case 1:
      //renewed fail
      #ifdef INFO
      Serial.println("Erreur : Echec du renouvelement du bail DHCP.");
      #endif
      return 0;

    case 2:
      //renewed success
      #ifdef INFO
      Serial.println("Renouvelement du bail DHCP.");
      //print your local IP address:
      Serial.print("Adresse IP : ");
      Serial.println(Ethernet.localIP());
      #endif
      udp.begin(udpPort);
      return 1;

    case 3:
      //rebind fail
      #ifdef INFO
      Serial.println("Erreur: Echec du rebind du bail DHCP. L'ESP va redemmarer.");
      #endif
      redemmarage();
    return 0;

    case 4:
      //rebind success
      #ifdef INFO
      Serial.println("Rebind du bail DHCP.");
      //print your local IP address:
      Serial.print("Adresse IP : ");
      Serial.println(Ethernet.localIP());
      #endif
      udp.begin(udpPort);
      return 1;

    default:
      //nothing happened
      return 1;
  }
}



//----------------------Capture et envoi de la photo---------------------------

boolean envoiPhoto() {
  // Gere la prise d'image et l'envoi de la photo

  // Aquisition de la capture
  camera_fb_t * fb = NULL;
  fb = esp_camera_fb_get();

  // Verification de la capture, redemmarage sinon
  if(!fb) {
    #ifdef INFO
    Serial.println("La capture a ratée, l'appareil va redemarrer");
    #endif
    redemmarage();
  }
  #ifdef INFO
  Serial.println("Capture prise !");
  #endif

  //Variables
  uint8_t *fbBuf = fb->buf; // Pointeur vers le binaire du fichier image
  size_t fbLen = fb->len; // Longueur
  uint8_t bufferReception[BUFFER_LEN]; // Buffer des paquet entrants

  // Determination de la procedure d'envoi
  #ifdef INFO
  Serial.println("Taille image : " + String(fbLen) + " octets");
  #endif
  unsigned int nbPaquets = fbLen / tailleData;
  if (fbLen % tailleData > 0) nbPaquets++;

  #ifdef INFO
  unsigned long int temps_envoi = millis();
  #endif

  // Compteur de retry, si on arrive pas a avoir de reponse positive du server
  // au bout de ABANDON fois, on abandonne.
  uint8_t count = 0;

  // Envoi de la requete jusqu'à réponse positive
  // Si ca fait plus de ABANDON fois qu'on reesaye alors on abandonne
  while(!requeteServeur(nbPaquets)) if (count++>ABANDON) {
    esp_camera_fb_return(fb); // Pour liberer l'espace mémoire
    #ifdef INFO
    Serial.println("La connexion avec le serveur a échouée (" + String(millis()-temps_envoi) + " ms).");
    #endif
    return false;
  }

  #ifdef INFO
  Serial.println("Envoi en cours de " + String(nbPaquets) + " paquets au serveur");
  #endif

  // On envoie les paquets un à un
  for (unsigned int i=0; i<nbPaquets; i++)
    envoiPaquet(i, nbPaquets, fbBuf, fbLen);

  // Compteur de retry, si on arrive pas a avoir de reponse positive du server
  // au bout de ABANDON fois, on abandonne.
  count = 0;

  // Creation du tableau et de la variable qui stockeront les paquets nonRecus
  uint8_t nbManquants = 0;
  uint8_t * tableauNonRecus = (uint8_t *)malloc(nbPaquets);

  // On envoi l'indication de fin, si tout est bon on sortira du while, sinon
  // on renvoi les paquets perdus et on reessaye d'indiquer la fin
  while (!indictationFin(nbPaquets, &nbManquants, tableauNonRecus)) {

    // Si ça fait plus de ABANDON fois qu'on reéssaye alors on abandonne
    if (count++ > ABANDON) {
      esp_camera_fb_return(fb); // Pour liberer l'espace mémoire
      free(tableauNonRecus); // Pour liberer l'espace mémoire
      #ifdef INFO
      Serial.println("L'envoi à échoué (" + String(millis()-temps_envoi) + " ms).");
      #endif
      return false;
    }

    // Quand il manque des paquets, on les renvoie puis on remet à zero le
    // stockage des paquets perdu pour reessayer d'indiquer la fin
    if (nbManquants) {
      for (uint8_t i=0; i<nbManquants; i++) envoiPaquet(tableauNonRecus[i], nbPaquets, fbBuf, fbLen); // Envoi des paquets
      memset(tableauNonRecus, 0, nbManquants); // RAZ
      nbManquants = 0; // RAZ
    }
  }

  #ifdef INFO
  Serial.println("Envoi terminé (" + String(millis()-temps_envoi) + " ms)");
  #endif

  // Libère l'espace mémoire que prend la photo et le tableauNonRecus
  esp_camera_fb_return(fb);
  free(tableauNonRecus);

  return true;
}



//------------------------Requete d'envoi des données--------------------------

boolean requeteServeur(unsigned int nbPaquets) {
  /* Envoie une requete d'envoi de photo au serveur
     puis en attend la reponse, elle peut être positive, negative
     ou ne pas avoir lieu (timeout).
     Si la requette obtient un reponse, la fonction revoit true,
     sinon elle revoit false.                                      */

  #ifdef INFO
  // Envoi de la requete d'upload
  Serial.println("Envoi de la requete d'upload de l'image au serveur");
  #ifdef MORE_INFO
  Serial.println(" -> Id de la carte : " + String(ID) + ", Nombre de packets à envoyer : " + String(nbPaquets) + ", Taille du buffer : " + String(DATA_MAX));
  #endif
  #endif

  // Creation du byte array à envoyer
  uint8_t demande[8];
  demande[0] = CLIENT_TYPE;
  demande[1] = ID;
  demande[2] = 1;
  demande[3] = nbPaquets;
  uint32_t octetsDataMax = htonl(DATA_MAX);
  memcpy((demande+4), &octetsDataMax, sizeof(uint32_t));

  // Paquet de demande (suit le protocole indiqué)
  udp.beginPacket();
  udp.write(demande, 8);
  udp.endPacket();

  // Reponse sensées être reçue du serveur
  uint8_t reponse[9];
  reponse[0] = CLIENT_TYPE;
  reponse[1] = ID;
  reponse[2] = 1;
  reponse[3] = 0;
  reponse[4] = nbPaquets;
  memcpy((reponse+5), &octetsDataMax, sizeof(uint32_t));

  // Timer pour timeout
  unsigned long int timer = millis();

  while (true)
  {
    // Remise à zéro du buffer et mise en place de la reception
    memset(bufferReception, 0, BUFFER_LEN);
    uint16_t longueurReception = udp.parsePacket();
    if (longueurReception)
    {
      // Lecture des paquets receptionnés
      udp.read(bufferReception, longueurReception);

      if (comparePacket(bufferReception, reponse, 4) && comparePacket((bufferReception+4), (reponse+4), 4)) //On ne regarde que les paquets de réponse
      {
        if (!bufferReception[3]) // Reponse correspondant à une reponse positive
        {
          #ifdef INFO
          #ifdef MORE_INFO
          Serial.println(" <- Reponse positive du serveur, démarrage de l'envoi... (" + String(millis()-timer) + " ms)");
          #endif
          #endif
          return true;
        }
        else // Reponse correspondant à une reponse negative
        {
          #ifdef INFO
          #ifdef MORE_INFO
          Serial.println(" <- Reponse negative du serveur, tentative de réenvoi de la requete... (" + String(millis()-timer) + " ms)");
          #endif
          #endif
          delay(WAIT_TIME);
          return false;
        }
      }
    }
    // Delai de reponse expiré
    if (millis() > timer+TIMEOUT)
    {
      #ifdef INFO
      Serial.println(" >< Timeout, tentative de réenvoi de la requete...");
      #endif
      delay(WAIT_TIME);
      return false;
    }
  }
}



//-----------------------------Envoi d'un paquet-------------------------------

void envoiPaquet(unsigned int idP, unsigned int nbPaquets, uint8_t *fbBuf, size_t fbLen) {
  /* Envoie le packet numero idP au serveur.                            */

  #ifdef INFO
  #ifdef MORE_INFO
  Serial.println(" -> Envoi paquet " + String(idP+1) + "/" + String(nbPaquets));
  #endif
  #endif

  // Creation du byte array d'entete
  uint8_t entete[ENTETE_LEN];
  entete[0] = CLIENT_TYPE;
  entete[1] = ID;
  entete[2] = 2;
  entete[3] = idP;
  uint32_t octetsTailleData;

  // On se met à l'emplacement du buffer correspondant au paquet à envoyer
  fbBuf += idP * tailleData;

  // Envoi du paquet avec les données
  if (((idP+1)*tailleData) < fbLen) { // Envoi des premiers paquets
    octetsTailleData = htonl(tailleData);
    memcpy((entete+4), &octetsTailleData, sizeof(uint32_t)); // On integre la taille du paquet dans l'entete
    udp.beginPacket();
    udp.write(entete, 8);
    udp.write(fbBuf,tailleData);
    udp.endPacket();
  }
  else if (fbLen%tailleData > 0) { // Envoi du dernier paquet (car plus petit que les autres)
    size_t remainder = fbLen%tailleData;
    octetsTailleData = htonl(remainder);
    memcpy((entete+4), &octetsTailleData, sizeof(uint32_t)); // On integre la taille du paquet dans l'entete
    udp.beginPacket();
    udp.write(entete, 8);
    udp.write(fbBuf,remainder);
    udp.endPacket();
  }
}



//------------------------Requete d'envoi des données--------------------------

boolean indictationFin(unsigned int nbPaquets, uint8_t * nbManquants, uint8_t * tableauNonRecus) {
  /* Indique au serveur que l'esp a envoyé tout ses paquets
     puis en attend que le serveur lui reponde si il a reçu tout les
     paquets ou si il en manque quels sont les paquets manquants
     La fonction prend en argument un pointeur vers un tableau ou elle
     peut indiquer les id des paquets manquants et un pointeur vers
     le nombre de paquets manquants.
     La fonction renvoi false si timeout ou paquets manquants et true
     si tout est bon                                                   */
  // /!\ Le tableau nonRecus doit au moins avoir la taille nbPaquets

  #ifdef INFO
  // Envoi de la requete d'upload
  Serial.println("Envoi de l'indication de fin de l'envoi");
  #ifdef MORE_INFO
  Serial.println(" -> Fin de l'envoi de " + String(nbPaquets) + " paquets");
  #endif
  #endif

  // Creation de l'indication
  uint8_t demande[4];
  demande[0] = CLIENT_TYPE;
  demande[1] = ID;
  demande[2] = 3;
  demande[3] = nbPaquets;

  // Envoi de l'indication
  udp.beginPacket();
  udp.write(demande, 4);
  udp.endPacket();

  // Reponse sensées être reçue du serveur
  uint8_t reponse[3];
  reponse[0] = CLIENT_TYPE;
  reponse[1] = ID;
  reponse[2] = 3;

  // Timer pour timeout
  unsigned long int timer = millis();

  while (true)
  {
    // Remise à zéro du buffer et mise en place de la reception
    memset(bufferReception, 0, BUFFER_LEN);
    uint16_t longueurReception = udp.parsePacket();
    if (longueurReception)
    {
      // Lecture des paquets receptionnés
      udp.read(bufferReception, longueurReception);

      if (comparePacket(bufferReception, reponse, 3)) //On ne regarde que les paquets de réponse
      {
        if (!bufferReception[3]) // Si il y'a 0 paquets manquants (on a bien tout reçu)
        {
          #ifdef INFO
          #ifdef MORE_INFO
          Serial.println(" <- Tout à été reçu (" + String(millis()-timer) + " ms)");
          #endif
          #endif
          return true; // On retourne 0 paquets manquants
        }
        else // Tout les paquets n'ont pas été reçus
        {
          #ifdef INFO
          #ifdef MORE_INFO
          Serial.println(" <- Il manque " + String() + "paquets (" + String(millis()-timer) + " ms)");
          #endif
          #endif

          // On enregistre le nombre et les paquets manquants
          *nbManquants = bufferReception[3];
          memcpy(tableauNonRecus, (bufferReception+4), *nbManquants); // On copie dans le tableau les paquets manquants
          return false; // On retourne le nombre de paquets manquants
        }
      }
    }
    // Delai de reponse expiré
    if (millis() > timer+TIMEOUT)
    {
      #ifdef INFO
      Serial.println(" >< Timeout, tentative de réenvoi de l'indication...");
      #endif
      delay(WAIT_TIME);
      return false;
    }
  }
}



//---------------------------Comparaison de tableau----------------------------

boolean comparePacket(uint8_t *table1, uint8_t *table2, size_t len) {
  for (int i=0; i<len; i++)
    if (table1[i] != table2[i])
      return 0;
  return 1;
}
