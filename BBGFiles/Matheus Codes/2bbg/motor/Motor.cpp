#include "Motor.hpp"

Motor::Motor(const int sentidoRotacao, const char* motorFilepath) :
sentidoRotacao_(sentidoRotacao), 
pathM_(motorFilepath)
{
        std::cout << "Motor criado: " << std::endl;
        std::cout << "Sentido de Rotação: " << sentidoRotacao_ << std::endl;
        std::cout << "Filepath motor: " << pathM_ << std::endl;
        std::cout << "---------------------" << std::endl;        

}

void Motor::setPWM(int PWM)
{
	int file;	

	//checar resultados de entrada da função
	if(PWM * sentidoRotacao_ < 0)
	{
		std::cout << "Erro: setPWM: valores invalidos de pwm ou sentido de rotação!" << std::endl;	
		exit(EXIT_FAILURE);
	}

	file = open(pathM_, O_RDWR | O_CREAT);
	
	//checar se abriu o arquivo do motor com sucesso
	if(file < 0)
	{
		std::cout << "Erro: setPWM: fracasso ao abrir o arquivo de duty_cycle do motor!" << std::endl;
		exit(EXIT_FAILURE);
	}

	char string_PWM[20]; //20 - arbritario
	
	sprintf(string_PWM, "%d", PWM * sentidoRotacao_ + 1000000);
	write(file, string_PWM, sizeof(string_PWM));
	close(file);
}

void Motor::setPWM()
{
	int file;	

	//checar resultados de entrada da função
	if(PWM_ * sentidoRotacao_ < 0)
	{
		std::cout << "Erro: setPWM: valores invalidos de pwm ou sentido de rotação!" << std::endl;	
		exit(EXIT_FAILURE);
	}

	file = open(pathM_, O_RDWR | O_CREAT);
	
	//checar se abriu o arquivo do motor com sucesso
	if(file < 0)
	{
		std::cout << "Erro: setPWM: fracasso ao abrir o arquivo de duty_cycle do motor!" << std::endl;
		exit(EXIT_FAILURE);
	}

	char string_PWM[20]; //20 - arbritario	
	sprintf(string_PWM, "%d", PWM_ * sentidoRotacao_ + 1000000);

	write(file, string_PWM, sizeof(string_PWM));
	close(file);
}

int Motor::convertVelocity_PWM(double velocity)
{
	int PWM = velocity * (9.54/103*10000);
	PWM_ = PWM; //para setar internamente. Retornar PWM para fins de debug
	return PWM;
}

void Motor::pararMotor()
{
	setPWM(convertVelocity_PWM(0.0));
}
