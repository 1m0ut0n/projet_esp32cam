/*  Coté serveur de la communication avec les esp
 *  N'integre pour l'instant que les esp pour la photo avec l'IA
 *  Comportement actuel :
 *  Recoit et repond aux demandes, receptionne les paquets et enregistre
 *  l'image.
 *  Si pendant un enregistrement le serveur recoit une nouvelle demande, il
 *  abandonnera la première et écrasera l'ancien fichier
 *  Amelioration a faire :
 *  - Permettre à plusieurs clients d'envoyer les photos en même temps
 *    -> Creer un tableau de taille max de client simultané
 *    -> Avoir une fonction qui retourne un espace vide du tableau
 *    -> Avoir une fonction qui donne l'index du client dans le tableau en
 *     fonction de son id
 *    -> Si un client demande et qu'on rempli déjà le tableau on refuse
 *  - Rajouter la taille des images
 */


#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

#include <iostream>

using namespace cv;

#define PORT    44444
#define DATA_MAX 1460

// Pour avoir plus d'information
//#define INFO

struct transfertEspPhoto {
  unsigned char idEsp;
  unsigned char nbPaquet;
  size_t tailleBuffer;
  uint8_t * img;
  //unsigned int carriage = 0;
  unsigned long int tailleImg;
};


//------------------Reponse à une demande d'envoi d'image----------------------

int reponseDemande(uint8_t *buffer, struct transfertEspPhoto *esp, int sockfd, struct sockaddr_in cliaddr) {

  printf("Reception d'une image\n");
  #ifdef INFO
  printf("-> Reception d'une requette de transfert d'image\n");
  #endif

  // On part du principe que tout se passerra bien
  // On peut le changer en cours de route si quelque chose se passe mal
  char ok = 0;

  // Enregistrement des données necessaires au transfert
  esp->idEsp = buffer[1]; // Id de l'esp
  esp->nbPaquet = buffer[3]; // Nombre de paquets necessaires à l'envoi
  uint32_t octetsTailleBuffer;
  memcpy(&octetsTailleBuffer, (buffer+4), sizeof(uint32_t));
  esp->tailleBuffer = ntohl(octetsTailleBuffer); // Taille du buffer de donnée sur chaque paquet
  esp->img = (uint8_t *)malloc((esp->nbPaquet)*(esp->tailleBuffer - 8)*sizeof(uint8_t)); // 8 est la taille de l'entete des paquets de données
  #ifdef INFO
  printf("  Enregistrement des données liées au transfert\n");
  #endif

  // Création de la reponse au client
  uint8_t reponse[9];
  reponse[0] = 1;
  reponse[1] = esp->idEsp;
  reponse[2] = 1;
  reponse[3] = ok;
  reponse[4] = esp->nbPaquet;
  uint32_t octetsTailleBuffer2 = htonl(esp->tailleBuffer);
  memcpy((reponse+5), &octetsTailleBuffer2, sizeof(uint32_t));
  // On aurait pu juste remmettre les données du buffer mais ça
  // nous permet d'être sûr qu'il n'y ai aucune erreur (notamment boutisme)

  // Envoi de la reponse
  sendto(sockfd, (const uint8_t *)reponse, 9, MSG_CONFIRM, (const struct sockaddr *) &cliaddr, sizeof(cliaddr));
  #ifdef INFO
  printf("<- Envoi d'une reponse positive\n");
  #endif

  return 1;
}


//----------------Enregistrement des donées et remise à zéro-------------------

int enregistrement(struct transfertEspPhoto *esp) {

  printf("Enregistrement de l'image");

  // Si une image existe déjà, on la supprime
  remove("reception/img.jpg");

  // Ouverture du fichier
  FILE *fichier;
  fichier = fopen("reception/img.jpg", "wb");
  if (fichier == NULL) {
    printf("échoué...\n");
    return 0;
  }

  // Ecriture
  fwrite(esp->img, esp->tailleImg, 1, fichier);
  fclose(fichier);

  printf(" réussi\n");

  // Remise à zéro de la structure esp
  esp->idEsp = 0;
  esp->nbPaquet = 0;
  esp->tailleBuffer = 0;
  free(esp->img);
  esp->tailleImg = 0;

  std::string image_path = samples::findFile("reception/img.jpg");
    Mat img = imread(image_path, IMREAD_COLOR);
    if(img.empty())
    {
        std::cout << "Could not read the image: " << image_path << std::endl;
        return 1;
    }
    imshow("Display window", img);
    int k = waitKey(5000);
    destroyAllWindows();
    waitKey(1); // Wait for a keystroke in the window
    if(k == 's')
    {
        imwrite("reception/img.png", img);
    }

  return 1;
}


//--------------------Reception d'un paquet de transfert-----------------------

int receptionPaquet(uint8_t *buffer, struct transfertEspPhoto *esp, int sockfd, struct sockaddr_in cliaddr) {

  // On vérifie qu'on communique toujours avec le même esp
  if (buffer[1] != esp->idEsp) {
    return 0;
  }

  // On part du principe que tout se passerra bien
  // On peut le changer en cours de route si quelque chose se passe mal
  char ack = 0;

  // Enregistrement des infos sur le paquet
  char idP = buffer[3]; // Id du paquet
  uint32_t octetstailleDonnees;
  memcpy(&octetstailleDonnees, (buffer+4), sizeof(uint32_t));
  int tailleDonnees = ntohl(octetstailleDonnees); // Taille du paquet

  #ifdef INFO
  printf("-> Reception du paquet %d/%d du transfert d'image\n", idP+1, esp->nbPaquet);
  #endif

  // On considère que tout les données du paquet on toutes pour taille la
  // tailleBuffer (sauf le dernier). Ca nous permet de réecrire au dessus
  // d'un autre paquet si besoin car il est facile de retrouver l'endroit
  // où celui-ci à été écrit.
  // Une autre methode (si on considère que chaque paquet peut avoir une)
  // taille différente serai de mettre en place un système de chariot
  // mais on ne pourrait pas réécrire des paquets déjà écrit (méthode append)

  // Endroit de l'image où l'on va écrire (donc içi tout les tailleBuffer)
  unsigned int carriage = idP * (esp->tailleBuffer-8);

  // Enregistrement des données
  memcpy((esp->img + carriage), (buffer+8), tailleDonnees);
  esp->tailleImg += tailleDonnees;

  // Création de la reponse au client
  uint8_t reponse[9];
  reponse[0] = 1;
  reponse[1] = esp->idEsp;
  reponse[2] = 2;
  reponse[3] = idP;
  reponse[4] = ack;
  uint32_t octetstailleDonnees2 = htonl(tailleDonnees);
  memcpy((reponse+5), &octetstailleDonnees2, sizeof(uint32_t));
  // On aurait pu juste remmettre les données du buffer mais ça
  // nous permet d'être sûr qu'il n'y ai aucune erreur (notamment boutisme)

  // Envoi de la reponse
  sendto(sockfd, (const uint8_t *)reponse, 9, MSG_CONFIRM, (const struct sockaddr *) &cliaddr, sizeof(cliaddr));
  #ifdef INFO
  printf("<- ACK paquet n°%d\n", idP+1);
  #endif

  // Si c'est le dernier paquet, on declenche l'enregistrement de l'image
  if (idP + 1 >= esp->nbPaquet)
    enregistrement(esp);

  return 1;
}


//----------------------------------Main---------------------------------------

int main() {


//---------------------------Setup protocole UDP-------------------------------

  printf("Initialisation du serveur\n");
  int sockfd;
  uint8_t buffer[DATA_MAX];
  struct sockaddr_in servaddr, cliaddr;

  // Creation du socket
  #ifdef INFO
  printf("   Création du socket\n");
  #endif
  if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
    perror("   >< Echec de la création du socket\n");
    exit(EXIT_FAILURE);
  }

  // Mise à zéro des espace mémoire
  memset(&servaddr, 0, sizeof(servaddr));
  memset(&cliaddr, 0, sizeof(cliaddr));

  // Filling server information
  servaddr.sin_family = AF_INET; // IPv4
  servaddr.sin_addr.s_addr = INADDR_ANY;
  servaddr.sin_port = htons(PORT);

  // Bind du socket avec l'adresse du serveur
  #ifdef INFO
  printf("   Bind du socket\n");
  #endif
  if (bind(sockfd, (const struct sockaddr *)&servaddr, sizeof(servaddr)) < 0)
  {
    perror("   >< Echec du bind\n");
    exit(EXIT_FAILURE);
  }

  printf("Initialisation terminée, le serveur est prêt !\n");


//----------------------------Actions du serveur-------------------------------

  // Variables
  int len, n;
  int run = 1;
  len = sizeof(cliaddr);  //len is value/resuslt
  struct transfertEspPhoto esp;
  esp.tailleImg = 0;
  memset(buffer, 0, DATA_MAX);

  while (run) { // Le serveur tourne à l'infini, mais on pourrait imaginer une action de fin grâce au run
    n = recvfrom(sockfd, (char *)buffer, DATA_MAX, 0, ( struct sockaddr *) &cliaddr, (socklen_t*)&len);
    if (n < 0) {
      perror("   >< Echec de la reception\n");
      exit(EXIT_FAILURE);
    }

    // Impression des données recue en hexa
    //printf("data : ");
    //for(size_t i=0; i<DATA_MAX; i++) printf("%02x ", buffer[i]);
    //printf("\n");

    if (buffer[0] == 1) // Cas des ESP photographes
    {
      if (buffer[2] == 2) {
        receptionPaquet(buffer, &esp, sockfd, cliaddr);
      }
      else if (buffer[2] == 1) { // Cas d'une reception de demande
        reponseDemande(buffer, &esp, sockfd, cliaddr);
      }
    }

    memset(buffer, 0, DATA_MAX);
  }

  return 0;
}
