#ifndef LQRMAT_HPP
#define LQRMAT_HPP

#include <iostream>
//biblioteca utilizada para realizar operacoes matriciais
#include <armadillo>
#include <string.h>

/*
As diretivas abaixo definem o tamanho da linha e coluna das
matrizes utilizadas no controle e de forma indireta na
dimensao dos vetores auxiliares utilizados na classe

i: linha
j: coluna
*/
#define iK1 4
#define jK1 3
#define iKx 4
#define jKx 12
#define iRef 3
#define jRef 1
#define icoefRef 3
#define jcoefRef 2
#define iSV 12
#define jSV 1
#define iTrim 4
#define jTrim 1
#define iError 4
#define jError 1
#define iOutputControl 4
#define jOutputControl 1

//Classe do controlador do tipo LQR

class LQR
{
	public:
	    /*
	    Construtor da classe. Recebe parametros gerados no MATLAB. Criado desta forma para que os
	    parametros sejam alterados conforme a vontade do usuario no MATLAB sem que seja necessario
	    recompilar o sistema na beagle bone a cada modificacao dos parametros abaixo.

	    O MATLAB envia as matrizes na forma de vetor, logo para receber os parametros no construtor,
	    as variaveis com o termo 'Vet' tambem sao vetores. A variavel dtMatlab e um escalar.
	    */
		LQR(double *K1VetMatlab, double *KxVetMatlab, double *CoefRefVetMatlab, double dtMatlab, double *trimVetMatlab);

		/*
		Apos a construcao das matrizes no 'formato C' estas matrizes sao convertidas para o 'formato
		armadillo'
		*/
		void initArmaMatrix();

		/*
		Inicializa os parametros das matrizes no 'formato C' e o escalar dt. Armazena nos vetores auxiliares
		os respectivos parametros para posteriormente serem convertidos no 'formato Armadillo'
		*/
		void initParameters(double *K1VetMatlab, double *KxVetMatlab, double *CoefRefVetMatlab, double dtMatlab, double *trimVetMatlab);

		/*
		Inicializa as matrizes de ganho K1 e Kx no 'formato C'. Armazenando o valor recebido pelo MATLAB
		num vetor auxiliar e depois construindo matrizes referentes as matrizes de ganho no 'formato C'
		*/
		void initMatrizesDeGanho(double *K1VetMatlab, double *KxVetMatlab);

		/*
		Inicializa a matriz que contem os coeficientes da expressao que representa a referencia
		para o controlador

		Esta matriz e construida no 'formato C'
		*/
		void initReferencia(double *CoefRefVetMatlab);

		/*
		Inicializa o parametro dt na classe com o valor de dt recebido atraves do MATLAB
		*/
		void init_dt(double dtMatlab);

		/*
		Inicializa a matriz de trimagem no 'formato C' utilizada no controlador através
		dos valores enviados pelo MATLAB. Depende da quantidade de motores utilizados no sistema.
		*/
        void initTrim(double *trimVetMatlab);

        /*
        atribuir iterativamente valors para mat armaSV
        A funcao abaixo converte a matriz SV (state variables) do 'formato C'
        para o 'formato Armadillo'. Esta funcao e usada separadamente pois
        a cada iteracao ela deve ser chamada para atualizar a matriz armadillo
        com a nova matriz SV recebida atraves do envio do MATLAB.

        O primeiro argumento e a matriz armadillo onde serao armazenados os
        valores da matriz SV no formato C. O segundo e terceiro argumentos
        sao, respectivamente, as dimensoes da linha e da coluna da matriz
        armadillo que e da mesma dimensao da matriz no formato C.
        */
        void setArmaMatrix(arma::mat &ArmaMatrix, size_t sizeLin, size_t sizeCol);

        /*
        Retorna os valores calculados pelo controlador. Funcao retorna vetor.
        */
		double* getOutputControl();

		/*
		Retorna a dimensao do vetor onde estao armazenados os calculos de
		saida do controlador.
		*/
        size_t getOutputCtlSize();

        /*
        Realiza o calculo do controle. Calculo da saida de fato do controlador.
        */
		void computeControl();

		/*
		Realiza o calculo da referencia a partir dos coeficientes do polinomio
		de referencia.
		*/
		void computeRef();

		/*
        Funcao componente da logica de montagem de dados apos recebimento de dado
        atraves do MATLAB. Constroi matriz no 'formato c'.

        Componente do pacote de funcoes constituido de:
            * recvMatrizVetorizada( ... )
            * constructMatrix ( ... )

		*/
        void constructMatrix(size_t linSize, size_t colSize, const char *matrix); //do matlab a matrix vem vetorizada

		/*
		Funcao utilizada para debugar. Imprime exclusivamente matrizes de ganho K1 ou Kx
		na forma 'formatada' ou 'por extenso'.

		seja o exemplo matriz 2x3
		'formatada':

                        a1  a2  a3
                        a4  a5  a6

		'por extenso':
		[1][1] = a1
		[1][2] = a2
		[1][3] = a3
		[2][1] = a4
		etc...

		*/
		void imprimirMat(char flag, char qualMatriz);

        /*
        Para visualizacao e debug a funcao imprime os principais parametros
        relativos ao controle na tela.
        */
        void printParameters();

        /*
        Para visualizacao e debug. Esta funcao imprime um vetor existente
        na tela ao passar o nome do vetor e o seu respectivo tamanho.
        */
		void printVet(double *vet, size_t sizeVet);

		/*

		*/
		void recvMatrizVetorizada(double *K1vetFromMatlab, double *K2vetFromMatlab, size_t size_K1vetFromMatlab, size_t size_KxvetFromMatlab);
		void recvMatrizVetorizada(double *vetFromMatlab, size_t size_vetFromMatlab, const char *paramControle);
		void recvMatrizVetorizada(double *vetFromMatlab, size_t size_vetFromMatlab, double *vet2Store, size_t size_vet2Store);
		void setStateVariables(double *XXvetFromMatlab, size_t sizeColXXvetFromMatlab);
		void vetorizarOutControl();

	private:
		double K1[iK1][jK1], Kx[iKx][jKx],
                        Ref[iRef][jRef], SV[iSV][jSV],
                        trim[iTrim][jTrim], coefRef[icoefRef][jcoefRef];

		double K1Vetorizado[iK1*jK1], KxVetorizado[iKx*jKx],
                        RefVetorizado[iRef*jRef],
                        coefRefVetorizado[icoefRef*jcoefRef],
                        SVVetorizado[iSV*jSV],
                        trimVetorizado[iTrim*jTrim],
                        OutputControlVetorizado[iOutputControl*jOutputControl];

    	double dt; //Control time difference
    	double t0 = 0; //tempo inicial de iteração
    	double t = 0; //tempo final de iteração

        arma::mat armaK1, armaKx; //matrizes de ganho
		arma::mat armaRef; //Reference for controller
		arma::mat armaSV; //State variables
    	arma::mat armaOutputControl; //Control Variables
		arma::mat armaError; //Global error for control
		arma::mat armaTrim; //Inital Velocity for control
    	arma::mat armaCoefRef; //POR QUESTOES DE DEBUG

        size_t sizeOutCtlVet;
};

#endif
