#include <iostream>
#include <unistd.h>
#include <stdio.h>
#include <sys/socket.h>
#include <stdlib.h>
#include <netinet/in.h>
#include <string.h>
#include <math.h>
#include <cmath>
#include <fstream>

#define PI 3.14159265
#define Lb 0.620 //m
#define DT 0.02 //s


#include "comm/Comm.hpp"
#include "emergency/Emergency.hpp"
#include "imu/MPU6050/MPU6050.h"

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

    //Cria instancia do IMU MPU6050
    MPU6050 mpu;

    //Inicializa MPU6050
    mpu.initialize();

    //Verifica se MPU foi inicializada corretamente
    if (mpu.testConnection())
    {
        std::cout << "Conecao com MPU iniciada com sucesso!" << std::endl;
    }
    else
    {
        std::cout << "Conecao com MPU falhou. Saindo do programa!" << std::endl;
        return -1;
    }

    //Parametros de offset da IMU
    int accel_offsets[3] = {-3037, -428, 2155};
    int gyro_offsets[3] = {58, 1, -39};

    //Setagem dos offsets
    mpu.setXAccelOffset(accel_offsets[0]);
    mpu.setYAccelOffset(accel_offsets[1]);
    mpu.setZAccelOffset(accel_offsets[2]);

    mpu.setXGyroOffset(gyro_offsets[0]);
    mpu.setYGyroOffset(gyro_offsets[1]);
    mpu.setZGyroOffset(gyro_offsets[2]);

    /*
    Variaveis onde serao armazenados os valores
    das aceleracoes do acelerometro e as velocidades
    angulares do giroscopio
    */
    double accx, accy, accz, gyrox, gyroy, gyroz;

    /**/
    double roll, pitch;

    /*******************ARQUIVO**********************/
    //Instanciar objeto para escrever em arquivo
    std::ofstream myfile;
    //Abrir arquivo/criar arquivo e abrir
    myfile.open("IMU_DADOS.txt");

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
  
    double dtFromMatlab;    
    double XXVet[iSV * jSV]; //conferir depois

    double *ptrOutCntToMatlab;

    //Este parametro e enviado pelo MATLAB
    double numeroIteracoes;

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
    while (true)
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
        comm.receberDados(&dtFromMatlab, sizeof(dtFromMatlab));        
        comm.receberDados(&numeroIteracoes, sizeof(numeroIteracoes));

        /*
    Por questoes de debug sao imprimidos na tela os valores recebidos
atraves do MATLAB. Para as funcoes abaixo sao passados como argumentos
uma string representando uma mensagem para ser impressa na tela,
os vetor de dados que serao impressos e o tamanho do vetor.

*/

        //debug        
        comm.printData("dt from matlab", &dtFromMatlab, 1);        
        comm.printData("numero de Iteracoes from matlab", &numeroIteracoes, 1);


        /*
    Abaixo e iniciado as iteracoes em que o controle sera computado.
Onde serao os obtidos os valores de velocidade e convertidos para
valores de PWM. Os valores de velocidade sao enviados para o MATLAB
e os valores de PWM sao enviados para os motores.

*/

        for (int i = 0; i < numeroIteracoes; i++)
        {

            /*
    Os valores de estado sao recebidos apos serem enviados
pelo MATLAB. Argumentos: onde serao armazenados os dados
e o tamanho do vetor onde serao armazenados os dados.

*/
            //ler XX
            //comm.receberDados(XXVet, sizeof(XXVet));
            //alterar XX          

           
            //Ler os valores de aceleracao e vel. angular e armazenar nas variaveis por referencia

            mpu.getMotion6(&accx, &accy, &accz, &gyrox, &gyroy, &gyroz);

            roll = atan2(accy, (sqrt(accx * accx + accz * accz))) * 180.0 / PI;
            pitch = atan2((-accx), (sqrt(accy * accy + accz * accz))) * 180.0 / PI;

            /************************ARQUIVO**********************************************/

            myfile << accx << '\t' << accx << '\t' << accz << '\t'
                   << gyrox << '\t' << gyroy << '\t' << gyroz  << '\t' 
                   << roll  << '\t' << pitch << std::endl;

            XXVet[0] = accx;
            XXVet[1] = accx;
            XXVet[2] = accz;
            XXVet[3] = gyrox;
            XXVet[4] = gyroy;
            XXVet[5] = roll;
            XXVet[6] = pitch;            

            //enviar XX
            comm.enviarDados(XXVet, sizeof(XXVet));        
            }
             /*
    Apos finalizar as iteracoes a conexao e encerrada com o cliente.
*/
        comm.closeSocket();  

        /*
    Neste ponto o laço infindavel ira retornar ao inicio
aceitando uma nova conexao caso exista e o processo se
reiniciara. Caso nao haja conexao para ser aceita o programa
ficara em espera sem realizar nenhuma acao.
*/

        /************************ARQUIVO***************/
        myfile.close();
    }

    return 0;
}

