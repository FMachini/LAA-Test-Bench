#ifndef EMERGENCY_H
#define EMERGENCY_H

#include <stdio.h>
#include <signal.h>
#include <stdlib.h>

#include "../motor/Motor.hpp"

/*
    Função que chamada quando o sinal enviado para o programa quando as
teclas Ctrl+c sao pressionadas. Esta funcao abre os arquivos que contem
os valores de pwm que sao enviados para os motores e escreve neles
valor de PWM que faz os motores pararem
*/
void paradaDeEmergencia(int sig);

#endif
