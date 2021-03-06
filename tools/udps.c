#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>

#include <arpa/inet.h>

void error(char *msg) {
    perror(msg);
    exit(1);
}

int main(int argc, char *argv[]) {
    int sockfd, portno, len;
    socklen_t clilen;
    char buffer[256];
    struct sockaddr_in6 serv_addr, cli_addr;
    int n;
    char client_addr_ipv6[100];

    if (argc < 2) {
        fprintf(stderr, "Usage: %s 1234\n", argv[0]);
        exit(0);
    }

    printf("\nIPv6 UDP Server Started...\n");
    
    //Sockets Layer Call: socket()
    sockfd = socket(AF_INET6, SOCK_DGRAM, IPPROTO_UDP);
    if (sockfd < 0)
        error("ERROR opening socket");

    bzero((char *) &serv_addr, sizeof(serv_addr));
    portno = atoi(argv[1]);
	// server.sin6_scope_id=if_nametoindex("eth0");
	//serv_addr.sin6_scope_id = 2;
    serv_addr.sin6_flowinfo = 0;
    serv_addr.sin6_family = AF_INET6;
    serv_addr.sin6_addr = in6addr_any;
	//inet_pton(AF_INET6, "fe80::f168:3982:82e5:7018", (void *)&serv_addr.sin6_addr.s6_addr);
    serv_addr.sin6_port = htons(portno);

    
    //Sockets Layer Call: bind()
    if (bind(sockfd, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0)
        error("ERROR on binding");

    //Sockets Layer Call: listen()
    listen(sockfd, 5);
    
	//while (1) 
   {
    memset(buffer,0, 256);
    
    //Sockets Layer Call: recv()
    n = recvfrom(sockfd, buffer, sizeof(buffer), 0, (struct sockaddr *) &cli_addr, &len);
    if (n < 0)
        error("ERROR reading from socket");

    printf("Message from client: %s\n", buffer);

    //Sockets Layer Call: send()
   n = sendto(sockfd, "Server got your message", 23+1, 0, (struct sockaddr *) &cli_addr, sizeof(serv_addr));
    if (n < 0)
        error("ERROR writing to socket");
    }
    //Sockets Layer Call: close()
	printf("close socket ...\n");
    close(sockfd);
    
    return 0;
}


