// Client side implementation of UDP client-server model
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>

#define PORT 44444

// Driver code
int main() {
    int sockfd;
    char message[100];
    struct sockaddr_in cliaddr, servaddr;

    // Creating socket file descriptor
    if ( (sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0 ) {
        perror("socket creation failed");
        exit(EXIT_FAILURE);
    }

    memset(&cliaddr, 0, sizeof(cliaddr));
    memset(&servaddr, 0, sizeof(servaddr));

    // Filling client and server information
    cliaddr.sin_family = AF_INET;
    cliaddr.sin_port = htons(PORT);
    cliaddr.sin_addr.s_addr = inet_addr("192.168.10.231");
    servaddr.sin_family = AF_INET;
    servaddr.sin_port = htons(PORT);
    servaddr.sin_addr.s_addr = INADDR_ANY;

    // Bind du socket avec l'adresse du serveur
    // (a commenter si on met le serveur de reception en même temps)

    int n, len;

    while (1) {
        printf("Envoi : ");
        fgets(message, 100, stdin);
        message[strcspn(message, "\r\n")] = '\0';
        sendto(sockfd, (const char *)message, strlen(message),
            MSG_CONFIRM, (const struct sockaddr *) &cliaddr,
                sizeof(cliaddr));
        printf("> Message sent : %s\n", message);
    }

    close(sockfd);
    return 0;
}
