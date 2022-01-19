#ifndef COMM_HPP
#define COMM_HPP

#include <iostream>
#include <unistd.h>
#include <stdio.h>
#include <sys/socket.h>
#include <stdlib.h>
#include <netinet/in.h>
#include <string.h>

#define PORT 8080

class Comm
{
	public:
		Comm();
		//inicializa todos os parametros para comunicação via TCP/Ip
		
		~Comm();
		//encerrar recursos utilizados pelo socket
		
		void createSocket();
		void bindSetup();
		void binding();
		void listening();
		void accepting();
		void receberDados(void* Guardar, size_t lenGuardar);
		void enviarDados(void* Enviar, size_t lenEnviar);
		void closeSocket();
                
                void printData(const char *label, double *vetor, size_t sizeVet);
			
	private:
		int server_fd, new_socket;
		struct sockaddr_in address;
		int opt = 1;
		int addrlen = sizeof(address);
		double buffer;  
};

#endif
