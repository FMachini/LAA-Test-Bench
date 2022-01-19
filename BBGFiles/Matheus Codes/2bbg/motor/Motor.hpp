#ifndef MOTOR_HPP
#define MOTOR_HPP

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

#define default_M1_path "/sys/class/pwm/pwmchip0/pwm0/duty_cycle"
#define default_M2_path "/sys/class/pwm/pwmchip3/pwm0/duty_cycle"
#define default_M3_path "/sys/class/pwm/pwmchip1/pwm1/duty_cycle"
#define default_M4_path "/sys/class/pwm/pwmchip1/pwm0/duty_cycle"

#define DEBUG_default_M1_path "M1_duty_cycle.txt"
#define DEBUG_default_M2_path "M2_duty_cycle.txt"
#define DEBUG_default_M3_path "M3_duty_cycle.txt"
#define DEBUG_default_M4_path "M4_duty_cycle.txt"

#define default_period_BBG 1000000

#define default_sentidoRotacaoM1 -1
#define default_sentidoRotacaoM2 -1
#define default_sentidoRotacaoM3 +1
#define default_sentidoRotacaoM4 +1

class Motor
{
	public:
		Motor(const int sentidoRotacao, const char* motorFilepath);
		//inicializa os atributos principais do motor		

		void setPWM(int PWM);
		//checar se pwm é positivo senao encerrar - erro!

		void setPWM();
		//ja utiliza o valor PWM_ gravado internamente

		int convertVelocity_PWM(double velocity);
		//converte a velocidade (output do controlador) em pwm

		void pararMotor();
		//encerra o movimento de rotação do motor. Depende do valor que esta setado o periodo do PWM 
		//na beagle bone. Por padrão é utilizado o valor de 1000000

                
	private:
		const char* pathM_;
		const int sentidoRotacao_;
		int PWM_;
};

#endif
