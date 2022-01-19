#include <iostream>
#include <unistd.h>
#include <stdio.h>
#include <sys/socket.h>
#include <stdlib.h>
#include <netinet/in.h>
#include <string.h>

#include "control/lqrmat.hpp"
#include "comm/Comm.hpp"
#include "motor/Motor.hpp"
#include "emergency/Emergency.hpp"

int main() 
{

 Comm comm;
  Motor motor1(default_sentidoRotacaoM1, default_M1_path), 
	motor2(default_sentidoRotacaoM2, default_M2_path), 
	motor3(default_sentidoRotacaoM3, default_M3_path), 
	motor4(default_sentidoRotacaoM4, default_M4_path);
 
          //parada de emergencia - ctrl + c
        signal(SIGINT, paradaDeEmergencia);

 
        double K1Vet[iK1*jK1];
        double KxVet[iKx*jKx];
        double CoefRefVet[icoefRef*jcoefRef];
        double dtFromMatlab;
        double trimVet[iTrim*jTrim];
        double XXVet[iSV*jSV];
        
        double *ptrOutCntToMatlab;
        
        int numeroIteracoes = 1000;

    //tentativa de fazer o server rodar sempre e aceitar um cliente e depois desconectar
    while(true)
    {
	comm.accepting();
                
        //receber parametros iniciais do matlab
        comm.receberDados(K1Vet, sizeof(K1Vet));
        comm.receberDados(KxVet, sizeof(KxVet));
        comm.receberDados(CoefRefVet, sizeof(CoefRefVet));
        comm.receberDados(&dtFromMatlab, sizeof(dtFromMatlab));
        comm.receberDados(trimVet, sizeof(trimVet));
         
        //debug
        comm.printData("Vetor K1 from matlab", K1Vet, iK1*jK1);
        comm.printData("Vetor Kx from matlab", KxVet, iKx*jKx);
        comm.printData("Vetor de Coeficientes de referencia from matlab", CoefRefVet, icoefRef*jcoefRef); 
        comm.printData("dt from matlab", &dtFromMatlab, 1);
        comm.printData("Vetor trim from matlab", trimVet, iTrim*jTrim);
        
        LQR lqr(K1Vet, KxVet, CoefRefVet, dtFromMatlab, trimVet);
        
        
        for(int i = 0; i < numeroIteracoes; i++)
        {
            //ler XX
            comm.receberDados(XXVet, sizeof(XXVet));

            //calcular controle
            lqr.setStateVariables(XXVet, iSV*jSV);
            lqr.computeControl();

            //obter saida do controlador
            ptrOutCntToMatlab = lqr.getOutputControl();

	    //convenção vem do matlab conforme foi modelado anteriormente
	    //trim[0] -- motor 3
	    //trim[1] -- motor 1	
	    //trim[2] -- motor 4
	    //trim[3] -- motor 2
	    //estar definições foram realizadas de forma arbritaria. A regra que deve
	    //ser satisfeita é que os motores 1 e 2 recebam velocidades negativas
	    //e os motores 3 e 4 recebam velocidades positivas
	    motor1.convertVelocity_PWM(ptrOutCntToMatlab[1]);
	    motor2.convertVelocity_PWM(ptrOutCntToMatlab[3]);
	    motor3.convertVelocity_PWM(ptrOutCntToMatlab[0]);
	    motor4.convertVelocity_PWM(ptrOutCntToMatlab[2]);

            //enviar dados de PWM para os motores
	    motor1.setPWM();
	    motor2.setPWM();
	    motor3.setPWM();
	    motor4.setPWM();

	    //vetor double de quatro posicoes - velocidade de 4 motores;
            //enviar controle
            comm.enviarDados(ptrOutCntToMatlab, lqr.getOutputCtlSize());
        }

        comm.closeSocket();

	motor1.pararMotor();
	motor2.pararMotor();
	motor3.pararMotor();
	motor4.pararMotor();
	printf("Motors closed.\n");

    }

    return 0;
}

