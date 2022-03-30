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
 *  - Rajouter la taille des images (surement pas necessaire)
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

// Structure qui enregistre les données liées à un envoi
struct transfertEspPhoto {

  // Indentification
  unsigned char idEsp;

  // Reception
  unsigned char nbPaquets = 0;
  uint8_t * paquetsRecus;
  size_t tailleBuffer = 0;

  // Image et enregitrement
  uint8_t * img;
  unsigned long int tailleImg = 0;
};



//------------------Reponse à une demande d'envoi d'image----------------------

int reponseDemande(uint8_t *buffer, struct transfertEspPhoto *esp, int sockfd, struct sockaddr_in cliaddr) {
  // Fonction qui intervient en cas de reception d'une demande d'envoi d'image.
  // Elle initialise l'envoi et repond à l'esp

  printf("Reception d'une image\n");
  #ifdef INFO
  printf("-> Reception d'une requette de transfert d'image\n");
  #endif

  // On part du principe que la reponse sera positive
  // On peut le changer en cours de route si quelque chose se passe mal
  char ok = 0;

  // Enregistrement des données necessaires au transfert
  esp->idEsp = buffer[1]; // Id de l'esp
  esp->nbPaquets = buffer[3]; // Nombre de paquets necessaires à l'envoi
  uint32_t octetsTailleBuffer;
  memcpy(&octetsTailleBuffer, (buffer+4), sizeof(uint32_t));
  esp->tailleBuffer = ntohl(octetsTailleBuffer); // Taille du buffer de donnée sur chaque paquet (sauf dernier)
  esp->img = (uint8_t *)malloc((esp->nbPaquets)*(esp->tailleBuffer - 8)*sizeof(uint8_t)); // 8 est la taille de l'entete des paquets de données
  esp->paquetsRecus = (uint8_t *)malloc(esp->nbPaquets*sizeof(uint8_t)); // On met un tableau indiquant pour chaque paquet si il a été recu ou non
  #ifdef INFO
  printf("  Enregistrement des données liées au transfert\n");
  #endif

  // Création de la reponse au client
  uint8_t reponse[9];
  reponse[0] = 1; // Protocole
  reponse[1] = esp->idEsp; // id esp
  reponse[2] = 1; // Type de reponse
  reponse[3] = ok; // Accepte ou non
  reponse[4] = esp->nbPaquets; // Nombre de paquet
  uint32_t octetsTailleBuffer2 = htonl(esp->tailleBuffer);
  memcpy((reponse+5), &octetsTailleBuffer2, sizeof(uint32_t)); // Taille buffer
  // On aurait pu juste remmettre les données du buffer mais ça
  // nous permet d'être sûr qu'il n'y ai aucune erreur (notamment boutisme)

  // Envoi de la reponse
  sendto(sockfd, (const uint8_t *)reponse, 9, MSG_CONFIRM, (const struct sockaddr *) &cliaddr, sizeof(cliaddr));
  #ifdef INFO
  printf("<- Envoi d'une reponse positive\n");
  #endif

  return 1;
}



//--------------------Reception d'un paquet de transfert-----------------------

int receptionPaquet(uint8_t *buffer, struct transfertEspPhoto *esp, int sockfd, struct sockaddr_in cliaddr) {
  // Fonction appelée lors de la reception d'un paquet, elle enregistre les
  // data dans le buffer et les données liées dans la classe esp

  // On vérifie qu'on communique toujours avec le même esp
  if (buffer[1] != esp->idEsp) {
    return 0;
  }

  // Enregistrement des infos sur le paquet
  char idP = buffer[3]; // Id du paquet
  uint32_t octetstailleDonnees;
  memcpy(&octetstailleDonnees, (buffer+4), sizeof(uint32_t));
  int tailleDonnees = ntohl(octetstailleDonnees); // Taille du paquet

  #ifdef INFO
  printf("-> Reception du paquet %d/%d du transfert d'image\n", idP+1, esp->nbPaquets);
  #endif

  // Ici on considère que tout les données du paquet on toutes pour taille la
  // tailleBuffer (sauf le dernier). Ca nous permet de réecrire au dessus
  // d'un autre paquet si besoin car il est facile de retrouver l'endroit
  // où celui-ci à été écrit.
  // Une autre methode (si on considère que chaque paquet peut avoir une
  // taille différente) serai de mettre en place un système de chariot
  // mais on ne pourrait pas réécrire des paquets déjà écrit (méthode append)

  // Endroit de l'image où l'on va écrire (donc içi tout les tailleBuffer)
  unsigned int carriage = idP * (esp->tailleBuffer-8);

  // Enregistrement des données au bon endroit
  memcpy((esp->img + carriage), (buffer+8), tailleDonnees);

  // On ajoute le paquet reçu au tableau de paquetsRecus
  esp->paquetsRecus[idP] = 1;

  // Si c'est le dernier paquet, on calcule la taille de l'images
  // En effet, seul le dernier paquet à une taille spécifique, tout les autres
  // on forcement une taille de tailleBuffer-8
  if (idP + 1 >= esp->nbPaquets)
    esp->tailleImg = ((esp->nbPaquets-1)*(esp->tailleBuffer-8)) + octetstailleDonnees;

  return 1;
}



//----------------Enregistrement des donées et remise à zéro-------------------

int enregistrement(struct transfertEspPhoto *esp) {
  // Fonction appelée

  printf("Enregistrement de l'image");

  // Si une image existe déjà, on la supprime
  remove("reception/img.jpg");

  // Ouverture du fichier
  FILE *fichier;
  fichier = fopen("reception/img.jpg", "wb");
  if (fichier == NULL) {
    printf(" échoué...\n");
    return 0;
  }

  // Ecriture du fichier
  fwrite(esp->img, esp->tailleImg, 1, fichier);
  fclose(fichier); // Fermeture

  printf(" réussi\n");

  // Remise à zéro de la structure esp
  esp->idEsp = 0;
  esp->nbPaquets = 0;
  esp->tailleBuffer = 0;
  free(esp->img);
  free(esp->paquetsRecus);
  esp->tailleImg = 0;


  // Affichage OpenCV (je comprend pas tout mais c'est un code trouvé dans
  // la doc d'OpenCV, désolé j'ai plus le lien)
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



//------------------------Reponse à un paquet de fin---------------------------

int reponseFin(uint8_t *buffer, struct transfertEspPhoto *esp, int sockfd, struct sockaddr_in cliaddr) {
  // Fonction qui intervient en cas de reception d'une indication de fin
  // d'envoi. Elle verifie qu'elle à bien reçue tout les paquets et repond
  // au serveur.

  #ifdef INFO
  printf("-> Reception de l'indication de la fin d'envoi\n");
  #endif

  // Verification des paquets reçus
  uint8_t manquants = 0; // Pour connaitre le nombre de paquets manquants
  uint8_t * nonRecus = (uint8_t *)malloc(esp->nbPaquets); // Liste des paquets non reçus
  for (uint8_t idP=0; idP<esp->nbPaquets; idP++) {
    if (!esp->paquetsRecus[idP]) { // Si on a pas reçu le paquet idP
      nonRecus[manquants] = idP;
      manquants++;
    }
  }

  // Création de la reponse au client
  uint8_t * reponse = (uint8_t *)malloc(4 + manquants); // Taille dynamique depandant du nonbre de paquets manquants
  reponse[0] = 1; // Protocole
  reponse[1] = esp->idEsp; // id esp
  reponse[2] = 3; // Type de reponse
  reponse[3] = manquants; // Nombre de paquet manquants
  memcpy((reponse+4), nonRecus, manquants); // On copie la liste de non recu

  // Envoi de la reponse
  sendto(sockfd, (const uint8_t *)reponse, 4+manquants, MSG_CONFIRM, (const struct sockaddr *) &cliaddr, sizeof(cliaddr));
  #ifdef INFO
  printf("<- Reponse : il manque %d paquets\n", manquants);
  #endif

  // On libere la place prise par nonRecus et reponse
  free(nonRecus);
  free(reponse);

  // Si on a reçu tout les paquets, on engage l'enregitrement
  if (!manquants) enregistrement(esp);

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
      else if (buffer[2] == 3) {
        reponseFin(buffer, &esp, sockfd, cliaddr);
      }
    }

    memset(buffer, 0, DATA_MAX);
  }

  return 0;
}
