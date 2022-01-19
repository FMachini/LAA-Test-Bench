#ifndef MOTOR_HPP
#define MOTOR_HPP

/*
Observacao: esta classe abstrai um motor. Esta classe abstrai os
atributos e acoes de um motor. Desta forma, no projeto como se usam
quatro motores, entao sao necessarios quatro objetos desta classe
pois desta forma o comportamento e atributo de cada motor estarao
representados.
*/

#include <iostream>
#include <stdlib.h> //exit(...)
#include <cstdio> //sprintf

//file I/O
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdio.h>
#include <sys/types.h>
#include <unistd.h>
#include <stdio.h>

//signal handler - ctrl + C - parada de emergencia.
#include <signal.h>
#include <stdlib.h>

/*
As diretivas abaixo definem o caminho utilizado no projeto atualmente.
Sao criadas estas diretivas a fim de facilitar a programacao nas situacoes
de acesso ao caminho do arquivo. Estes arquivos sao aqueles que estao
contidos os valores dos sinais PWM que serao enviados para os motores.
*/
#define default_M1_path "/sys/class/pwm/pwmchip0/pwm0/duty_cycle"
#define default_M2_path "/sys/class/pwm/pwmchip3/pwm0/duty_cycle"
#define default_M3_path "/sys/class/pwm/pwmchip1/pwm1/duty_cycle"
#define default_M4_path "/sys/class/pwm/pwmchip1/pwm0/duty_cycle"

/*
Diretivas utilizadas simular o que seria escrito no arquivo dentro da beaglebone
porem num arquivo de texto no computador que esta executando o programa a fim
de debugar o programa. Utilize estas diretivas somente nas finalidades de debug
fora do sistema da beagle bone.
*/
#define DEBUG_default_M1_path "M1_duty_cycle.txt"
#define DEBUG_default_M2_path "M2_duty_cycle.txt"
#define DEBUG_default_M3_path "M3_duty_cycle.txt"
#define DEBUG_default_M4_path "M4_duty_cycle.txt"

/*
Esta diretiva define o valor de duty cycle inicial do sinal PWM que e escrito
no arquivo quando a beaglebone e inicializada. Este valor nao produz rotacao
nos motores. A unidade do duty cycle e nanosegundos <!>
*/
#define default_dutyCycle_BBG 1000000

/*
Diretivas para facilitar o uso do sentido de rotacao de cada motor
definido de acordo com o projeto do controlador.
*/
#define default_sentidoRotacaoM1 -1
#define default_sentidoRotacaoM2 -1
#define default_sentidoRotacaoM3 +1
#define default_sentidoRotacaoM4 +1

class Motor
{
	public:

	    //inicializa os atributos principais do motor
		Motor(const int sentidoRotacao, const char* motorFilepath);

		/*
		Escreve no arquivo o valor do sinal PWM que se deseja enviar da
		beagle bone para o motor.
		O argumento da funcao e o valor de PWM que se deseja enviar para
		o motor.
		Esta funcao tambem checa se PWM é positivo senao encerrar - erro!
		*/
		void setPWM(int PWM);

        /*
        Função semelhante a 'void setPWM(int PWM)' porem nao e necessario
        enviar um valor de PWM como argumento pois esta funcao utiliza a
        variavel interna da classe PWM_ que por sua vez e atribuida um valor
        internamente na funcao 'int convertVelocity_PWM(double velocity)'.
        Desta forma o programador nao precisa se preocupar com o valor que
        deve ser passado para a funcao, pois a responsabilidade de enviar o
        parametro correto esta atruida a funcao mencionada anteriormente.
        */
		void setPWM();

		/*
		Esta funcao converte um valor de velocidade, em RPM, e o converte
		para um sinal de PWM, em nanosegundos, mais especificamente num
		valor de duty cycle. Este valor de PWM e armazenado
		na variavel interna PWM_ e para fins de debug ou para outro motivo
		o valor da conversao tambem e retornado pela funcao.

		A finalidade desta funcao e transformar o valor de velocidade calculado
		pelo controlador num sinal PWM para que o mesmo possa ser enviado para
		o motor. O valor de velocidade e enviado para o MATLAB.
		*/
		int convertVelocity_PWM(double velocity);

		/*
        Encerra o movimento de rotação do motor. Depende do valor que esta setado o duty cycle do PWM
		na beagle bone. Por padrão é utilizado o valor de 1000000.

		Funcao utilizada quando as iteracoes sao finalizadas e assim a funcao e utilizada para
		que pare o motor.
		*/
		void pararMotor();

	private:

	    /*
	    Variavel interna da classe para internalizar o caminho do arquivo que armazena
	    o sinal de PWM a ser enviado para o motor.
	    */
		const char* pathM_;

		/*
		Armazena internamente o sentido de rotacao do motor.
		*/
		const int sentidoRotacao_;

		/*
		Armazena internamente na classe o valor de PWM para ser enviado para o motor.
		*/
		int PWM_;
};

#endif
