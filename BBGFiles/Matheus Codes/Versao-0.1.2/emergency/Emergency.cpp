#include "Emergency.hpp"

void paradaDeEmergencia(int sig)
{
    /*
        Variaveis para armazenar o caminho do arquivo
    onde esta escrito o valor do sinal de PWM
    */
	int fileM1, fileM2, fileM3, fileM4;

    /*
        Abre os arquivos para escrever valores de PWM.
    Os arquivos sao abertos para leitura e escrita
    (argumento O_RDWR).Caso os arquivos por algum motivo
    nao existam eles serao criados (argumento O_CREAT).*/
	fileM1 = open(default_M1_path, O_RDWR | O_CREAT);
	fileM2 = open(default_M2_path, O_RDWR | O_CREAT);
	fileM3 = open(default_M3_path, O_RDWR | O_CREAT);
	fileM4 = open(default_M4_path, O_RDWR | O_CREAT);

	/*
	Variaveis onde sera escrito o valor de sinal PWM que para a rotacao dos motores e que sera escrito no arquivo para cada motor
	*/
	char string_PWM_M1[20], string_PWM_M2[20], string_PWM_M3[20], string_PWM_M4[20]; //20 - arbritario

	/*
	Escreve na variavel (primeiro argumento) um valor do tipo inteiro (segundo argumento)
	que e o valor do sinal (duty cycle) que para os motores (terceiro argumento)
	Obs.: este valor tem a mesma magnitude do sinal que eh enviado para os motores quando
	inicializa o sistema, uma vez que ao inicializar o sistema os motores estarao parados
	e continuaram desta forma ate que seja enviado um sinal de magnitude superior a esta
	magnitude inicial.
	*/
	sprintf(string_PWM_M1, "%d", default_dutyCycle_BBG);
	sprintf(string_PWM_M2, "%d", default_dutyCycle_BBG);
	sprintf(string_PWM_M3, "%d", default_dutyCycle_BBG);
	sprintf(string_PWM_M4, "%d", default_dutyCycle_BBG);

	/*
	Escreve no arquivo (primeiro argumento) o valor de PWM (segundo argumento),
	um tamanho tal de bytes (terceiro argumento). Em caso de erro a funcao de
	escrita retorna -1 e a mensagem eh exibida para o usuario para que o erro
	seja rastreado. Este procedimento ocorre para todos os motores.
	*/
	if(write(fileM1, string_PWM_M1, sizeof(string_PWM_M1)) < 0)
    {
     std::cout << "Erro na parada de emergencia. Erro ao escrever no motor1!" << std::endl;
    }
	close(fileM1);

	if(write(fileM2, string_PWM_M2, sizeof(string_PWM_M2)) < 0)
    {
     std::cout << "Erro na parada de emergencia. Erro ao escrever no motor2!" << std::endl;
    }
	close(fileM2);

	if(write(fileM3, string_PWM_M3, sizeof(string_PWM_M3)) < 0)
    {
     std::cout << "Erro na parada de emergencia. Erro ao escrever no motor3!" << std::endl;
    }
	close(fileM3);

	if(write(fileM4, string_PWM_M4, sizeof(string_PWM_M4)) < 0)
    {
     std::cout << "Erro na parada de emergencia. Erro ao escrever no motor4!" << std::endl;
    }
	close(fileM4);

	/*
	Ao fim da execucao desta funcao a mensagem e exibida para informar o usuario
	*/

    std::cout << "---------------------------------------------------" << std::endl;
	std::cout << "Ctrl+C pressionado! Parada de emergencia!" << std::endl;
	std::cout << "Parada de emergencia. Parada a rotação dos motores." << std::endl;
    std::cout << "---------------------------------------------------" << std::endl;

    /*
    Funcao exit(...) e utilizada para encerrar a execucao do programa do sistema
    do hardware in the loop na beagle bone

    EXIT_FAILURE e passado como argumento para a funcao exit para representar
    que o sistema (hardware in the loop) nao finalizou corretamente devido
    a algum motivo que disparou a parada de emergencia. EXIT_FAILTURE torna
    o codigo mais portavel.

     The use of EXIT_SUCCESS and EXIT_FAILURE is slightly more portable
       (to non-UNIX environments) than the use of 0 and some nonzero value
       like 1 or -1.  In particular, VMS uses a different convention.

    The C standard specifies two constants, EXIT_SUCCESS and
       EXIT_FAILURE, that may be passed to exit() to indicate successful or
       unsuccessful termination, respectively.

       fonte: http://man7.org/linux/man-pages/man3/exit.3.html
    */

	exit(EXIT_FAILURE);
}
