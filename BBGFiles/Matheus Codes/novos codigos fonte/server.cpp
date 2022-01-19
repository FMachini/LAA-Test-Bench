#include <iostream>
#include <unistd.h>
#include <stdio.h>
#include <sys/socket.h>
#include <stdlib.h>
#include <netinet/in.h>
#include <string.h>

#include "lqrmat.hpp"

#define PORT 8080

void printVet(double *vet, size_t sizeVet);

int main() 
{
    // server side
    //-Socket>>setsockopt>>bind>>listen>>accept>>send_recv

    int server_fd, new_socket, valread;
    struct sockaddr_in address;
    int opt = 1;
    int addrlen = sizeof(address);
    char buffer[1024] = {0};
    //const char *serverMsg = "hello from server! =)";
    double serverMsg = 42;

  // creatring socker file descriptor
  // socket creation
    if((server_fd = socket(AF_INET, SOCK_STREAM, 0)) < 0)
    {
        perror("cannot create socket");
        exit(EXIT_FAILURE);
    }

    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(PORT);

    //binding socket to the ip and port
    if(bind(server_fd, (struct sockaddr *) &address, sizeof(address)) < 0)
    {
        perror("cannot bind. Bind failed!");
        exit(EXIT_FAILURE);
    }

    if(listen(server_fd, 3) < 0)
    {
        perror("cannot listen");
        exit(EXIT_FAILURE);
    }


    //tentativa de fazer o server rodar sempre e aceitar um cliente e depois desconectar
    while(true)
    {
        if((new_socket = accept(server_fd, (struct sockaddr *) &address, (socklen_t *) &addrlen)) < 0)
        {
            perror("cannot accept");
            exit(EXIT_FAILURE);
        }
        printf("Connection stablished.\n\n");

        /*
        sleep(2);
        //send(new_socket, serverMsg, strlen(serverMsg), 0);
        send(new_socket, &serverMsg, sizeof(serverMsg), 0);
        printf("Hello message sent\n");
        */

        //~ double simPWM = 1000500;

        //testar enviar vetores
        /*
        double v[2] = {1.2, 2.3};
        double v1[3] = {100, 87.3, -473.444445555};
        double v2[10] = {52, 61, 5, 80, 95, 32, 66, 17, 23, 92};
        double v3[40] = {57, 92, 98, 6, 29, 25, 23, 53, 24, 98, 77, 6,
                         47, 49, 63, 61, 16, 46, 34, 85, 100, 38, 86,
                         20, 56, 91, 45, 44, 36, 35, 9, 36, 25, 32,
                         16, 1, 6, 85, 69, 38};

        double m[3][2] = {{3.14, 3.1415}, {9.81, 9.8019}, {0.707, 0.710}};
        double m1[2][3] = {{1.1, 2.2, 3.3}, {3.14, 4.13, 5.23}};
        double m2[1][4] = {1.0, 2.01, 3.02, 4.03};
        double m3[4][1] = {{23.23}, {123.23}, {423.21}, {412.21}};
        */

        //enviar de m a m3
        /*
        send(new_socket, m, sizeof(m), 0);
        send(new_socket, m1, sizeof(m1), 0);
        send(new_socket, m2, sizeof(m2), 0);
        send(new_socket, m3, sizeof(m3), 0);
        printf("send:\n");
        */

        /*
        while(true)
        {
        simPWM++;
        send(new_socket, &simPWM, sizeof(simPWM), 0);
        iW++;
        if(simPWM > 240) simPWM = 0;

        }
        */

        //testar varios tipos, int, float, double, string
        /*
        int i1 = 2008;
        float f1 = 3.14;
        double d1 = 3.1415;
        char *st = "abc";

        //send(new_socket, &i1, sizeof(i1), 0);
        //printf("send int:\n");

        // nao consigo lidar com float no matlab, somente double - 6/8/14
        //send(new_socket, &f1, sizeof(f1), 0);
        //printf("send float:\n");

        //send(new_socket, &d1, sizeof(d1), 0);
        //printf("send double:\n");

        //send(new_socket, st, strlen(st), 0);
        //printf("send string:\n");
        */


		/*
		//--------------------------------------------------------------
        //receber vetores e matrizes

        //tentativa de utilizar o imprime vetor para imprimir matrizes
        //->>> matlab nao envia matrizes, ele envia vetores

        //declarar
        double rm[3];
        double rm1[4];
        double rm2[4];
        double rm3[6];

        //receber
        recv(new_socket, rm, sizeof(rm), 0);
        recv(new_socket, rm1, sizeof(rm1), 0);
        recv(new_socket, rm2, sizeof(rm2), 0);
        recv(new_socket, rm3, sizeof(rm3), 0);

        

        //imprimir
        printf("rm - xx \n");
            printVet(rm, 3);
        printf("rm1 - sxx1 \n");
            printVet(rm1, 4);
        printf("rm2 - sxx2 \n");
            printVet(rm2, 4);
        printf("rm3 - sxx3 \n");
            printVet(rm3, 6);
		//--------------------------------------------------------------
		*/
				
		//~ recv(new_socket, &simPWM, sizeof(simPWM), 0);
        //~ printf("recvd: %f\n", simPWM);
        
        //receber parametros
        
        double K1Vet[iK1*jK1];
        double KxVet[iKx*jKx];
        double CoefRefVet[icoefRef*jcoefRef];
        double dtFromMatlab;
        double trimVet[iTrim*jTrim];
        double XXVet[iSV*jSV];
        
        double outputControlToMatlab[iOutputControl*jOutputControl];
        double *ptrOutCntToMatlab;
        
        recv(new_socket, K1Vet, sizeof(K1Vet), 0);
        recv(new_socket, KxVet, sizeof(KxVet), 0);
        recv(new_socket, CoefRefVet, sizeof(CoefRefVet), 0);
        recv(new_socket, &dtFromMatlab, sizeof(dtFromMatlab), 0);
        recv(new_socket, trimVet, sizeof(trimVet), 0);
        recv(new_socket, XXVet, sizeof(XXVet), 0);
        
        printVet(K1Vet, iK1*jK1);
        printVet(KxVet, iKx*jKx);
        printVet(CoefRefVet, icoefRef*jcoefRef); 
        printf("%f\n\n", dtFromMatlab);
        printVet(trimVet, iTrim*jTrim);
        
        LQR obj(K1Vet, KxVet, CoefRefVet, dtFromMatlab, trimVet);
        
        obj.setStateVariables(XXVet, iSV*jSV);
        obj.computeControl();
        
        ptrOutCntToMatlab = obj.getOutputControl();
               
				obj.imprimirMat('f', '1');
				obj.imprimirMat('f', 'x');  
				
				//
				//da pra fazer esse parametro ser enviado pelo matlab tambem. Dessa forma evitar recompilar o programa
				int numeroIteracoes = 1000;
				for(int i = 0; i < numeroIteracoes; i++)
				{
					//ler XX
					recv(new_socket, XXVet, sizeof(XXVet), 0);
					
					//calcular controle
					obj.setStateVariables(XXVet, iSV*jSV);
					obj.computeControl();
					
					//enviar controle
					ptrOutCntToMatlab = obj.getOutputControl();
					
					send(new_socket, ptrOutCntToMatlab, sizeof(double)*4, 0);
				}
				
				//~ double sedex = 12.01;
				//~ send(new_socket, &sedex, sizeof(sedex), 0);
				
				//~ std::cout << "enviei dado" << std::endl;
				//
				obj.printParameters();     


				//~ printVet(ptrOutCntToMatlab, iOutputControl*jOutputControl);

        close(new_socket);
        printf("Closing connection with the client.\n\n");
    }

    printf("Hello message sent\n");

    return 0;
}

void printVet(double *vet, size_t sizeVet)
{
    for(int i = 0; i < sizeVet; i++)
    {
        printf("%d: %f\n", i, *(vet++));
    }

    printf("\n\n");
}

