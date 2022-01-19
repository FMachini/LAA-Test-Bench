#include <iostream>
#include <unistd.h>
#include <stdio.h>
#include <sys/socket.h>
#include <stdlib.h>
#include <netinet/in.h>
#include <string.h>
#include <math.h>

#define PI 3.14159265
#define L 100 //mm
#define DT 0.02 //s

#include "control/lqrmat.hpp"
#include "comm/Comm.hpp"
#include "motor/Motor.hpp"
#include "emergency/Emergency.hpp"
#include "encoder/eqep.h"

int main()
{

/*
    Instancia objeto da classe de comunicação. O construtor
inicializa e configura a rede para comunicação entre MATLAB
(computador - cliente) e beagle bone (controlador - servidor).

    A classe configura os parametros de rede, configura o
beagle bone como servidor, envia e recebe dados, abre e encerra
conexão.

*/
 Comm comm;

/*
    Instancia objetos da classe dos motores. Responsaveis
por definir o caminho do arquivo de configuracao relativo
ao pwm do motor, definir o sentido de rotacao do motor,
converter valor de velocidade em pwm, enviar sinal para
definir o pwm enviado para o motor e encerrar a rotacao do
motor.

    Os argumentos "default" abaixo sao definidos no arquivo
de declaracoes da classe do motor: "Motor.hpp" <!>

*/

 Motor motor1(default_sentidoRotacaoM1, default_M1_path),
	motor2(default_sentidoRotacaoM2, default_M2_path),
	motor3(default_sentidoRotacaoM3, default_M3_path),
	motor4(default_sentidoRotacaoM4, default_M4_path);

/*
    A funcao abaixo e uma funcao da biblioteca c. Ela e
responsavel por tratar sinais recebidos. Um exemplo seria
a identificacao de um evento assincrono que ocorre fora do
programa quando uma tecla do teclado e pressionada.

    Neste sistema a funcao e utilizada para identificar
quando as teclas "Ctrl + c" sao pressionadas. Quando forem,
o sistema para imediatamente a rotacao dos motores e encerra
a execucao do programa. Podendo ser utilizado tanto para um
tipo de parada de emergencia ou para executar o programa por
outro motivo.

    Argumentos da funcao signal:

    SIGINT - sinal de interrupcao - geralmente ocorre devido
a acoes do usuario

    paradaDeEmergencia - e a funcao que sera invocada quando
o sinal e recebido quando as teclas Ctrl+c sao pressionadas.
A funcao encerra a rotacao dos motores.
funcao definida em: Emergency.cpp <!>

    Obs.: O sistema encerra automaticamente a rotacao dos
motores quando a iteracao e finalizada com sucesso.

*/
        //parada de emergencia - ctrl + c
        signal(SIGINT, paradaDeEmergencia);

 //cria instancia do encoder
 eQEP eqepVertical(eQEP2, eQEP::eQEP_Mode_Absolute);
 eQEP eqepHorizontal(eQEP1, eQEP::eQEP_Mode_Absolute);
 // Set the unit time period to 100,000,000 ns, or 0.1 seconds
 eqepVertical.set_period(100000000L);
 eqepHorizontal.set_period(100000000L);

 double eVertical_p0, eVertical_p1, eVertical_Ang, eHorizontal_p0, eHorizontal_p1, eHorizontal_Ang;

 //obter posicao ao iniciar sistema
 eVertical_p0 = eqepVertical.get_position();
 eHorizontal_p0 = eqepHorizontal.get_position();

/*
    Variaveis destinadas a armazenar dados enviados pelo
MATLAB.

    K1Vet: vetor onde sera escrito os valores da matriz
de ganho K1 apos ter sido vetorizada pelo MATLAB para
que seja enviado para o beagle bone.

    KxVet: vetor onde sera escrito os valores da matriz
de ganho K1 apos ter sido vetorizada pelo MATLAB para
que seja enviado para o beagle bone.

    CoefRefVet: vetor que armazena os coeficiente do
polinomio de referencia referente as coordenadas x, y
e z. Enviado atraves do MATLAB.

    dtFromMatlab: intervalo de tempo de iteracao. Enviado
atraves do MATLAB.

    trimVet: vetor que armazena os valores de trimagem.
Valores enviados atraves do MATLAB.

    XXVet: vetor que armazena os dados relativos ao estado
da aeronave. Enviados atraves do MATLAB.

    ptrOutCntToMatlab: ponteiro que a ponta para variavel
do tipo double. Utilizada para apontar para o vetor que
contem os valores calculdados de saida do controlador. Estes
valores contidos no vetor serao enviados para o MATLAB.

    numeroIteracoes: numero de iteracoes que ocorerrao. Valor
fixo determinado no programa. Somente no codigo-fonte. Este
mesmo valor deve ser o mesmo no codigo "cliente" do MATLAB.

*/
        double K1Vet[iK1*jK1];
        double KxVet[iKx*jKx];
        double CoefRefVet[icoefRef*jcoefRef];
        double dtFromMatlab;
        double trimVet[iTrim*jTrim];
        double XXVet[iSV*jSV];

        double *ptrOutCntToMatlab;

        int numeroIteracoes = 1000;

        double x, y, z;
        //para calculo de velocidade
        double x_old = 0, y_old = 0, z_old = 0;

        double vX, vY, vZ;

/*
    Este laco durante um numero infindavel de vezes.
O proposito e que o microcontrolador ("servidor") execute
continuamente. Sempre escutando/esperando por conexoes.
Quando uma conexao for escutada ela e aceitada pelo servidor
e assim o cliente estara conectado ao servidor, dados de
configuracao do controlador serao enviados, ocorrera um
'numeroIteracoes' de iteracoes, a conexao sera encerrada
pelo servidor e o servidor voltara ao estado de 'espera'
por novas conexoes.

    Em outras palavras, basta que o codigo seja iniciado uma vez
no microcontrolador e quando desejado basta executar a rotina no
MATLAB que a comunicacao sera estabelecida entre MATLAB e beaglebone
e o sistema atuara. Caso seja necessario realizar alteracoes na
rotina do MATLAB, o mesmo podera ser feito de forma independente do
programa executado no microcontrolador. Nao e necessario alterar o
programa no servidor nem recompila-lo.

*/
    while(true)
    {
/*
    Beagle bone aceita conexao e estabelece conexao com o cliente
*/
	comm.accepting();

/*
    As funcoes abaixo sao responsaveis por receber os parametros
iniciais relativos ao controlador. De forma generica, sao passados
para as funcoes onde se deseja escrever os dados e o tamanho da
variavel onde seram escritos os dados recebidos.

*/
        //receber parametros iniciais do MATLAB
        comm.receberDados(K1Vet, sizeof(K1Vet));
        comm.receberDados(KxVet, sizeof(KxVet));
        comm.receberDados(CoefRefVet, sizeof(CoefRefVet));
        comm.receberDados(&dtFromMatlab, sizeof(dtFromMatlab));
        comm.receberDados(trimVet, sizeof(trimVet));

/*
    Por questoes de debug sao imprimidos na tela os valores recebidos
atraves do MATLAB. Para as funcoes abaixo sao passados como argumentos
uma string representando uma mensagem para ser impressa na tela,
os vetor de dados que serao impressos e o tamanho do vetor.

*/

        //debug
        comm.printData("Vetor K1 from matlab", K1Vet, iK1*jK1);
        comm.printData("Vetor Kx from matlab", KxVet, iKx*jKx);
        comm.printData("Vetor de Coeficientes de referencia from matlab", CoefRefVet, icoefRef*jcoefRef);
        comm.printData("dt from matlab", &dtFromMatlab, 1);
        comm.printData("Vetor trim from matlab", trimVet, iTrim*jTrim);

/*
Apos os parametros do controlador serem recebidos. E instanciado um objeto
da classe referente ao controle LQR. O construtor configura a classe
com as matrizes de ganho, com os coeficientes dos polinomios relativos ao
sinal de referencia (onde dentro da classe o sinal de referencia e construido
atraves desses coeficientes), intervalo de tempo de iteracao e valores de
trimagem.

*/
        LQR lqr(K1Vet, KxVet, CoefRefVet, dtFromMatlab, trimVet);

/*
    Abaixo e iniciado as iteracoes em que o controle sera computado.
Onde serao os obtidos os valores de velocidade e convertidos para
valores de PWM. Os valores de velocidade sao enviados para o MATLAB
e os valores de PWM sao enviados para os motores.

*/

        for(int i = 0; i < numeroIteracoes; i++)
        {

/*
    Os valores de estado sao recebidos apos serem enviados
pelo MATLAB. Argumentos: onde serao armazenados os dados
e o tamanho do vetor onde serao armazenados os dados.

*/
            //ler XX
            comm.receberDados(XXVet, sizeof(XXVet));
            //alterar XX
            eVertical_p1 = eqepVertical.get_position();
            eVertical_Ang = ((eVertical_p1-eVertical_p0)*360/4000);
            eHorizontal_p1 = eqepHorizontal.get_position();
            eHorizontal_Ang = ((eHorizontal_p1-eHorizontal_p0)*360/4000);

            x = -L * cos(eHorizontal_Ang*(PI/180));
            y = L * sin(eHorizontal_Ang*(PI/180));
            z = L * sin(eVertical_Ang*(PI/180));

            vX = (x - x_old) / DT;
            vY = (y - y_old) / DT;
            vZ = (z - z_old) / DT;

            //reatribuicao de valor/atualizacao
            x_old = x;
            y_old = y;
            z_old = z;

            XXVet[0] = x;
            XXVet[1] = y;
            XXVet[2] = z;
            XXVet[3] = vX;
            XXVet[4] = vY;
            XXVet[5] = vZ;
            XXVet[6] = 0;
            XXVet[7] = 0;
            XXVet[8] = 0;
            XXVet[9] = 0;
            XXVet[10] = 0;
            XXVet[11] = 0;

/*
    Apos os valores serem recebidos, estes sao enviados para
o controlador. Argumentos: vetor contendo os valores de estado,
tamanho do vetor

    Em seguida sao realizados os calculos de controle.
*/
            //calcular controle
            lqr.setStateVariables(XXVet, iSV*jSV);
            lqr.computeControl();


/*
    O controlador processa os dados e obtem os dados de saida:
velocidade. Os valores de velocidade sao obtidos ao se passar
o endereco de memoria onde estes dados estao armazenados para
o ponteiro abaixo. Em outras palavras, e passado o endereco
vetor que contem os dados de saida do controlador para o ponteiro.

*/
            //obter saida do controlador
            ptrOutCntToMatlab = lqr.getOutputControl();

	    //convenção vem do MATLAB conforme foi modelado anteriormente
	    //trim[0] -- motor 3
	    //trim[1] -- motor 1
	    //trim[2] -- motor 4
	    //trim[3] -- motor 2
	    //estar definições foram realizadas de forma arbritaria. A regra que deve
	    //ser satisfeita é que os motores 1 e 2 recebam velocidades negativas
	    //e os motores 3 e 4 recebam velocidades positivas

/*
    Atraves das velocidades calculadas o proximo passo e converter estes valores
para os seus equivalentes em PWM para que sejam enviados estes valores para os
motores.

    Os valores convertidos de PWM sao armazenados de forma interna nos objetos
da classe de motores.

*/
	    motor1.convertVelocity_PWM(ptrOutCntToMatlab[1]);
	    motor2.convertVelocity_PWM(ptrOutCntToMatlab[3]);
	    motor3.convertVelocity_PWM(ptrOutCntToMatlab[0]);
	    motor4.convertVelocity_PWM(ptrOutCntToMatlab[2]);

/*
    A partir dos valores ja convertidos e armazenados
internamente nos objetos de motores sao enviados para
os motores os sinais de PWM.

*/

            //enviar dados de PWM para os motores
	    motor1.setPWM();
	    motor2.setPWM();
	    motor3.setPWM();
	    motor4.setPWM();

	    //vetor double de quatro posicoes - velocidade de 4 motores;
            //enviar controle

/*
    Os dados de velocidade sao enviados para o MATLAB.
Argumentos: vetor (implicito) contendo os dados,
tamanho do vetor que ira ser enviado. Nota: a funcao
getOutputCtlSize retorna o tamanho do vetor que contem
os dados de saida do controlador. <!>
*/
            comm.enviarDados(ptrOutCntToMatlab, lqr.getOutputCtlSize());
        }

/*
    Apos finalizar as iteracoes a conexao e encerrada com o cliente.
*/
        comm.closeSocket();

/*
    Caso a iteracao tenha ocorrido com sucesso os motores sao
comandados a pararem.

*/
	motor1.pararMotor();
	motor2.pararMotor();
	motor3.pararMotor();
	motor4.pararMotor();
    std::cout << "Motors closed." << std::endl;

/*
    Neste ponto o laço infindavel ira retornar ao inicio
aceitando uma nova conexao caso exista e o processo se
reiniciara. Caso nao haja conexao para ser aceita o programa
ficara em espera sem realizar nenhuma acao.

*/

    }

    return 0;
}

