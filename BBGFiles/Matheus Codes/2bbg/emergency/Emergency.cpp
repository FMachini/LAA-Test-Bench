#include "Emergency.hpp"

void paradaDeEmergencia(int sig)
{
	int fileM1, fileM2, fileM3, fileM4;

	fileM1 = open(default_M1_path, O_RDWR | O_CREAT);
	fileM2 = open(default_M2_path, O_RDWR | O_CREAT);
	fileM3 = open(default_M3_path, O_RDWR | O_CREAT);
	fileM4 = open(default_M4_path, O_RDWR | O_CREAT);	

	char string_PWM_M1[20], string_PWM_M2[20], string_PWM_M3[20], string_PWM_M4[20]; //20 - arbritario
	
	sprintf(string_PWM_M1, "%d", default_period_BBG);
	sprintf(string_PWM_M2, "%d", default_period_BBG);
	sprintf(string_PWM_M3, "%d", default_period_BBG);
	sprintf(string_PWM_M4, "%d", default_period_BBG);

	write(fileM1, string_PWM_M1, sizeof(string_PWM_M1));
	close(fileM1);

	write(fileM2, string_PWM_M2, sizeof(string_PWM_M2));
	close(fileM2);

	write(fileM3, string_PWM_M3, sizeof(string_PWM_M3));
	close(fileM3);

	write(fileM4, string_PWM_M4, sizeof(string_PWM_M4));
	close(fileM4);

	std::cout << "Ctrl+C pressionado! Parada de emergencia!" << std::endl;
	std::cout << "Parada de emergencia. Parada a rotação dos motores." << std::endl;

	exit(EXIT_FAILURE);
}
