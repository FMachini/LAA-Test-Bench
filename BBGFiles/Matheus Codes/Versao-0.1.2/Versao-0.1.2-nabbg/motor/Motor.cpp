#include "Motor.hpp"

/*
Construtor da classe motor. Recebe o valor do sentido de rotacao
e o caminho do arquivo que armazena o sinal de PWM.

Nota: so ha esse construtor pra evitar que seja construido um objeto
sem que seja inicializado os parametros cruciais do motor
*/

Motor::Motor(const int sentidoRotacao, const char *motorFilepath) : sentidoRotacao_(sentidoRotacao),
																																		pathM_(motorFilepath)
{
	std::cout << "Motor criado: " << std::endl;
	std::cout << "Sentido de Rotação: " << sentidoRotacao_ << std::endl;
	std::cout << "Filepath motor: " << pathM_ << std::endl;
	std::cout << "---------------------" << std::endl;
}

/*
Recebe valor de PWM para ser escrito no arquivo para que seja enviado
para o motor.
*/

void Motor::setPWM(int PWM)
{
	/*
	Variavel que representa o arquivo aberto (file descriptor)
	*/
	int file;

	//checar resultados de entrada da função
	if (PWM * sentidoRotacao_ < 0)
	{
		std::cout << "Erro: setPWM: valores invalidos de pwm ou sentido de rotação!" << std::endl;

		//encerra a execucao do programa
		exit(EXIT_FAILURE);
	}

	/*
    Abre o arquivo enderecado pelo caminho 'pathM_'. Abre o arquivo para leitura e escrita
    e caso o arquivo nao exista ele sera criado. O retorno da funcao interna do linux
    open retorna um file descriptor para o 'file' fazendo que esta variavel seja uma representacao
    abstrata do arquivo.
    */
	file = open(pathM_, O_RDWR | O_CREAT);

	//checar se abriu o arquivo do motor com sucesso
	if (file < 0)
	{
		std::cout << "Erro: setPWM: fracasso ao abrir o arquivo de duty_cycle do motor!" << std::endl;

		//encerra a execucao do programa
		exit(EXIT_FAILURE);
	}

	/*
	Esta variavel armazenara o sinal do PWM tratado e preparado para
	ser escrito no arquivo. O sinal de PWM pode ser negativo pois a funcao
	'int Motor::convertVelocity_PWM(double velocity)' podera retornar um valor
	negativo caso a velocidade seja negativa. Logo o sentido de rotacao
	armazenado na variavel sentidoRotacao_ trata o sinal para que o valor
	a ser escrito seja positivo. E acrescentado um valor constante de
	default_dutyCycle_BBG (definido como uma diretiva em Motor.hpp)
	este e um valor de duty cycle onde os motores nao tem potencia eletrica
	suficiente para rotacionar.
	*/
	char string_PWM[20]; //20 - arbritario

	/*
	Escreve um valor inteiro resultante da expressao abaixo na variavel string_PWM
	*/
	sprintf(string_PWM, "%d", PWM * sentidoRotacao_ + default_dutyCycle_BBG);

	/*
	Escreve no arquivo o valor do sinal PWM
	*/
	write(file, string_PWM, sizeof(string_PWM));

	/*
	Fecha o arquivo
	*/
	close(file);
}

/*
Funcao semelhante a void Motor::setPWM(int PWM), porem neste caso
nao e necessario passar como parametro o valor de PWM pois este valor
ja foi armazenado na variavel PWM_ pela
funcao int Motor::convertVelocity_PWM(double velocity)

*/
void Motor::setPWM()
{
	int file;

	//checar resultados de entrada da função
	if (PWM_ * sentidoRotacao_ < 0)
	{
		std::cout << "Erro: setPWM: valores invalidos de pwm ou sentido de rotação!" << std::endl;
		exit(EXIT_FAILURE);
	}

	file = open(pathM_, O_RDWR | O_CREAT);

	//checar se abriu o arquivo do motor com sucesso
	if (file < 0)
	{
		std::cout << "Erro: setPWM: fracasso ao abrir o arquivo de duty_cycle do motor!" << std::endl;
		exit(EXIT_FAILURE);
	}

	char string_PWM[20]; //20 - arbritario
	sprintf(string_PWM, "%d", PWM_ * sentidoRotacao_ + default_dutyCycle_BBG);

	write(file, string_PWM, sizeof(string_PWM));
	close(file);
}

/*
Converte o valor de velocidade calculado como sendo
a saida do controlador e converte esse valor para
um sinal PWM nao totalmente tratado. Este tratamento ocorre
posteriomente no fluxo de execucao do sistema. Veja
funcao void Motor::setPWM({  | int PWM})
*/
int Motor::convertVelocity_PWM(double velocity)
{
	/*
    9.54/103*10000 e uma constante responsavel pela transferencia
    de valor em velocidade para PWM 'nao tratado'
    */
	int PWM = velocity * (9.54 / 103 * 10000);
	PWM_ = PWM; //para setar internamente. Retornar PWM para fins de debug
	return PWM;
}

/*
Esta funcao para a rotacao do motor ao chamar a funcao
de convertar velocidade zero num valor de PWM para
ser enviado ao motor e assim para-lo.
*/
void Motor::pararMotor()
{
	setPWM(convertVelocity_PWM(0.0));
}
