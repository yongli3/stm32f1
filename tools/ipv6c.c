#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>

void error(char *msg) {
    perror(msg);
    exit(0);
}

int main(int argc, char *argv[]) {
    int sockfd, portno, n;
    struct sockaddr_in6 serv_addr;
    struct hostent *server;
    char buffer[256] = "This is a string from client!";

    if (argc < 3) {
        fprintf(stderr, "Usage: %s fe80::a200:0:0:2 1234 \n", argv[0]);
        exit(0);
    }
    portno = atoi(argv[2]);

    printf("\nIPv6 TCP Client Started...\n");
    
    //Sockets Layer Call: socket()
    sockfd = socket(AF_INET6, SOCK_STREAM, 0);
    if (sockfd < 0)
        error("ERROR opening socket");

    //Sockets Layer Call: gethostbyname2()
    server = gethostbyname2(argv[1],AF_INET6);
    if (server == NULL) {
        fprintf(stderr, "ERROR, no such host\n");
        exit(0);
    }

    memset((char *) &serv_addr, 0, sizeof(serv_addr));
    serv_addr.sin6_flowinfo = 0;
    serv_addr.sin6_family = AF_INET6;
	// /proc/net/igmp6
    serv_addr.sin6_scope_id = 2;
    memmove((char *) &serv_addr.sin6_addr.s6_addr, (char *) server->h_addr, server->h_length);
    serv_addr.sin6_port = htons(portno);

    //Sockets Layer Call: connect()
    if (connect(sockfd, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0)
        error("ERROR connecting");

    
    //Sockets Layer Call: send()
    n = send(sockfd,buffer, strlen(buffer)+1, 0);
    if (n < 0)
        error("ERROR writing to socket");

    memset(buffer, 0, 256);
    
    //Sockets Layer Call: recv()
    n = recv(sockfd, buffer, 255, 0);
    if (n < 0)
        error("ERROR reading from socket");
    printf("Message from server: %s\n", buffer);

    //Sockets Layer Call: close()
    close(sockfd);
        
    return 0;
}

