#include <netdb.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>

#define MAX 128
#define PORT 8888
#define SA struct sockaddr

void TokenizeStringToFloats(char str[], float currents[]){
    char * pch;
    pch = strtok (str,",");
    int i = 0;
    while (pch != NULL)
    {
        currents[i] = strtof (pch, NULL);
        i++;
        pch = strtok (NULL, ",");
    }
}

int main()
{
	int sockfd, connfd;
	struct sockaddr_in servaddr, cli;
	//3
	//float states[11]; //states + 1 

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
	servaddr.sin_addr.s_addr = inet_addr("192.168.1.4"); 
	servaddr.sin_port = htons(PORT);

	// connect the client socket to server socket
	if (connect(sockfd, (SA*)&servaddr, sizeof(servaddr)) != 0) {
		printf("connection with the server failed...\n");
		exit(0);
	}
	else
		printf("connected to the server..\n");

	char buff[MAX];
	//cannot use spaces or special characters in the string message as the program otherwise freaks out
	//torques to currents, assumes torques is an array of floats
	//3
	//const_wheels = 0.083;
	//sprintf(send_buff, "%f,%f,%f", torques[0]/const_wheels, torques[1]/const_wheels, torques[2]/const_wheels);
	char send_buff[MAX] = "<{0,0.5,0}>"; //edit this to what the actual output buffor will be
	        
	while(1){
		bzero(buff, sizeof(buff));

		//send string torques, ESP8266 <- PC
		write(sockfd, send_buff, sizeof(send_buff));
		
		//receive string states, ESP8266 -> PC
		read(sockfd, buff, sizeof(buff));
		printf("%s \n", buff);	
		
		//assume tokenization on ,
		//3
		//TokenizeStringToFloats(buff, states)
		//send states further with ROS to the controller
	}

	close(sockfd);
}

//gcc PC_testing_1D.c -o pc_testing
