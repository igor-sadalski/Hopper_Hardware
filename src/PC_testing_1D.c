#include <netdb.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>

#define MAX 80
#define PORT 8888
#define SA struct sockaddr

int main()
{
	int sockfd, connfd;
	struct sockaddr_in servaddr, cli;

	// socket create and verification
	sockfd = socket(AF_INET, SOCK_STREAM, 0);
	if (sockfd == -1) {
		printf("socket creation failed...\n");
		exit(0);
	}
	else
		printf("Socket successfully created..\n");
	bzero(&servaddr, sizeof(servaddr));

	// assign IP, PORT
	servaddr.sin_family = AF_INET;
	servaddr.sin_addr.s_addr = inet_addr("192.168.1.2"); 
	servaddr.sin_port = htons(PORT);

	// connect the client socket to server socket
	if (connect(sockfd, (SA*)&servaddr, sizeof(servaddr)) != 0) {
		printf("connection with the server failed...\n");
		exit(0);
	}
	else
		printf("connected to the server..\n");

	// function for chat
	char buff[MAX];
	char send_buff[10] = "<from PC>"; //edit this to what the actual output buffor will be
	        
	while(1){
		bzero(buff, sizeof(buff));
		
		receive string states, ESP8266 -> PC
		read(sockfd, buff, sizeof(buff));
		printf("%s \n", buff);
		
		//send string torques, ESP8266 <- PC
		write(sockfd, send_buff, sizeof(send_buff));		
	}

	close(sockfd);
}
