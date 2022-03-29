// Client side implementation of UDP client-server model
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>

#define PORT     44444
#define MAXLINE 1024

// Driver code
int main() {
    int sockfd;
    char buffer[MAXLINE];
    struct sockaddr_in     servaddr, cliaddr;

    // Creating socket file descriptor
    if ( (sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0 ) {
        perror("socket creation failed");
        exit(EXIT_FAILURE);
    }

    memset(&servaddr, 0, sizeof(servaddr));
    memset(&cliaddr, 0, sizeof(cliaddr));

    // Filling server information
    servaddr.sin_family = AF_INET;
    servaddr.sin_port = htons(PORT);
    servaddr.sin_addr.s_addr = INADDR_ANY;

    // Bind du socket avec l'adresse du serveur
    if (bind(sockfd, (const struct sockaddr *)&servaddr, sizeof(servaddr)) < 0)
    {
      perror("Echec du bind");
      exit(EXIT_FAILURE);
    }

    int n, len;

    while (1) {
      n = recvfrom(sockfd, (char *)buffer, MAXLINE,
                  MSG_WAITALL, (struct sockaddr *) &cliaddr,
                  &len);
      buffer[n] = '\0';

      printf("Reception :\n> Message : %s\n", buffer);
    }

    close(sockfd);
    return 0;
}
