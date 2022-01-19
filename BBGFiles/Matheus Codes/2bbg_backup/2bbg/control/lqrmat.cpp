#include "lqrmat.hpp"

//nota TRABALHA COM LOGICA DE MATRIZ NA MAO PARA TRATAR O VETOR QUE VEM DO MATLAB
//DEPOIS QUE TRATAR TEM QUE USAR O ARMADILLO PARA LIDAR COM MATRIZES (IMPRIMIR, MULTIPLICAR BLABLABLA)

//onde parei: 
/*
 * 
 */ 

//somente para debugar me apague depois
//===========================================================
//~ void printVet(double *vet, size_t sizeVet)
//~ {
    //~ for(size_t i = 0; i < sizeVet; i++)
    //~ {
        //~ printf("%lu: %f\n", i, *(vet++));
    //~ }

    //~ printf("\n\n");
//~ }
//===========================================================

LQR::LQR(double *K1VetMatlab, double *KxVetMatlab, double *CoefRefVetMatlab, double dtMatlab, double *trimVetMatlab)
{
	//initMatrix();
	initParameters(K1VetMatlab, KxVetMatlab, CoefRefVetMatlab, dtMatlab, trimVetMatlab);
	
}

void LQR::initArmaMatrix()
{
	//Define o tamanho de todas matrizes e as inicializa com zero	
	armaK1.zeros(iK1, jK1); 
	armaKx.zeros(iKx, jKx);	
	armaRef.zeros(iRef, jRef);
	armaSV.zeros(iSV, jSV);
	armaTrim.zeros(iTrim, jTrim);
	armaError.zeros(iError, jError);
	armaOutputControl.zeros(iOutputControl, jOutputControl);
	
	//POR QUESTOES DE DEBUG
	armaCoefRef.zeros(icoefRef, jcoefRef);
	
	//Inicializar matrizes "fixas"
	
	//atribuir valores para K1 no escopo do armadillo
	for(size_t j = 0; j < jK1; j++)
	{
		for(size_t i = 0; i < iK1; i++)
		{
			armaK1(i, j) = K1[i][j];
		}
	}
	
	//atribuir valores para Kx no escopo do armadillo
	for(size_t j = 0; j < jKx; j++)
	{
		for(size_t i = 0; i < iKx; i++)
		{
			armaKx(i, j) = Kx[i][j];
		}
	}	
	
	//atribuir valores para trim no escopo do armadillo
	for(size_t j = 0; j < jTrim; j++)
	{
		for(size_t i = 0; i < iTrim; i++)
		{
			armaTrim(i, j) = trim[i][j];
		}
	}	
	
	//POR QUESTOES DE DEBUG
	//atribuir valores para COEFREF no escopo do armadillo
	for(size_t j = 0; j < jcoefRef; j++)
	{
		for(size_t i = 0; i < icoefRef; i++)
		{
			armaCoefRef(i, j) = coefRef[i][j];
		}
	}	
		
}

void LQR::initParameters(double *K1VetMatlab, double *KxVetMatlab, double *CoefRefVetMatlab, double dtMatlab, double *trimVetMatlab)
{
	//as funções de init inicializam as matrizes na forma pura em C
	//já a função initArma converte essas matrizes para o formato armadillo
	
	//k1 e kx
	initMatrizesDeGanho(K1VetMatlab, KxVetMatlab);
	
	//referencia
	initReferencia(CoefRefVetMatlab);
	
	//tempo dt
	init_dt(dtMatlab);
			
	//trim
	initTrim(trimVetMatlab);
	
	//inicilizar as matrizes convertidas para armadillo
	//inicializa matrizes todas com zeros
	//inicializa matrizes "fixas"
	//matrizes "variaveis" são atualizadas em outro local
	initArmaMatrix();
	
	computeRef();
	
}

void LQR::initMatrizesDeGanho(double *K1VetMatlab, double *KxVetMatlab)
{
	//~ recvMatrizVetorizada(K1VetMatlab, KxVetMatlab, iK1*jK1, iKx*jKx);
	
	recvMatrizVetorizada(K1VetMatlab, iK1*jK1, "K1");
	recvMatrizVetorizada(KxVetMatlab, iKx*jKx, "Kx");
	
	constructMatrix(iK1, jK1, "K1");	
	constructMatrix(iKx, jKx, "Kx");
	
}

void LQR::printParameters()
{
	std::cout << "K1:" << std::endl;
	std::cout << armaK1 << std::endl;
	std::cout << "KX:" << std::endl;
	std::cout << armaKx << std::endl;
	std::cout << "REF:" << std::endl;
	std::cout << armaRef << std::endl;
	std::cout << "coefREF:" << std::endl;
	std::cout << armaCoefRef << std::endl;
	std::cout << "DT\n" << std::endl;
	std::cout << dt << std::endl;
	std::cout << "SV" << std::endl;
	std::cout << armaSV << std::endl;
	std::cout << "TRIM" << std::endl;
	std::cout << armaTrim << std::endl;
	
	std::cout << "OUTPUT CONTROL" << std::endl;
	std::cout << armaOutputControl << std::endl;	
	
	
}

void LQR::constructMatrix(size_t linSize, size_t colSize, const char *matrix)
{
	//transformar vetor em matriz
	// i - linha
	// j - coluna
		
	//arg char *matrix = para voce especificar qual matriz vc quer montar
		
	int aux = 0;
	
	bool compareStringK1, compareStringKx, compareStringRef, 
			 compareStringCoefRef, compareStringSV, compareStringTrim;
	
	compareStringK1 = true; 
	compareStringKx = true; 
	compareStringRef = true;
	compareStringCoefRef = true;
	compareStringSV = true;
	compareStringTrim = true;
	
	compareStringK1 = strcmp(matrix, "K1");
	compareStringKx = strcmp(matrix, "Kx");
	compareStringRef = strcmp(matrix, "Ref");
	compareStringCoefRef = strcmp(matrix, "CoefRef");
	compareStringSV = strcmp(matrix, "SV");
	compareStringTrim = strcmp(matrix, "trim");
	
	if(!(compareStringK1))
	{	
		for(size_t j = 0; j < colSize; j++)
		{
			for(size_t i = 0; i < linSize; i++)
			{
				K1[i][j] = K1Vetorizado[aux];
				aux++;
			}
		}	
	}
	
	if(!(compareStringKx))
	{
		for(size_t j = 0; j < colSize; j++)
		{
			for(size_t i = 0; i < linSize; i++)
			{
				Kx[i][j] = KxVetorizado[aux];
				aux++;
			}
		}		
	}
	
	if(!(compareStringRef))
	{	
		for(size_t j = 0; j < colSize; j++)
		{
			for(size_t i = 0; i < linSize; i++)
			{
				Ref[i][j] = RefVetorizado[aux];
				aux++;
			}
		}	
	}
	
	if(!(compareStringCoefRef))
	{	
		for(size_t j = 0; j < colSize; j++)
		{
			for(size_t i = 0; i < linSize; i++)
			{
				coefRef[i][j] = coefRefVetorizado[aux];
				aux++;
			}
		}	
	}
	
	if(!(compareStringSV))
	{	
		for(size_t j = 0; j < colSize; j++)
		{
			for(size_t i = 0; i < linSize; i++)
			{
				SV[i][j] = SVVetorizado[aux];
				aux++;
			}
		}	
	}
	
	if(!(compareStringTrim))
	{	
		for(size_t j = 0; j < colSize; j++)
		{
			for(size_t i = 0; i < linSize; i++)
			{
				trim[i][j] = trimVetorizado[aux];
				aux++;
			}
		}	
	}
	
}

void LQR::recvMatrizVetorizada(double *K1vetFromMatlab, double *KxvetFromMatlab, size_t size_K1vetFromMatlab, size_t size_KxvetFromMatlab)
{
	//atribui o vetor recebido enviado pelo matlab para um vetor temporario dentro da classe
	for(size_t i = 0; i < size_K1vetFromMatlab; i++)
	{
			K1Vetorizado[i] = K1vetFromMatlab[i];
	}
	
	for(size_t i = 0; i < size_KxvetFromMatlab; i++)
	{
			KxVetorizado[i] = KxvetFromMatlab[i];
	}
	
}

//pensar numa forma de fazer a recv receber, vetor de origem, tamanho e vetor de destino e tamanho


//alterar para função generica
void LQR::recvMatrizVetorizada(double *vetFromMatlab, size_t size_vetFromMatlab, const char *paramControle)
{
	//atribui o vetor recebido enviado pelo matlab para um vetor temporario dentro da classe

	bool compareStringK1, compareStringKx, compareStringRef, 
			 compareStringCoefRef, compareStringSV, compareStringTrim; 
	
	compareStringK1 = true; 
	compareStringKx = true; 
	compareStringRef = true;
	compareStringCoefRef = true;
	compareStringSV = true;
	compareStringTrim = true;
	
	compareStringK1 = strcmp(paramControle, "K1");
	compareStringKx = strcmp(paramControle, "Kx");
	compareStringRef = strcmp(paramControle, "Ref");
	compareStringCoefRef = strcmp(paramControle, "CoefRef");
	compareStringSV = strcmp(paramControle, "SV");
	compareStringTrim = strcmp(paramControle, "trim");

	if(!compareStringK1)
	{
		for(size_t i = 0; i < size_vetFromMatlab; i++)
		{
			K1Vetorizado[i] = vetFromMatlab[i];
		}
	}
	
	if(!compareStringKx)
	{
		for(size_t i = 0; i < size_vetFromMatlab; i++)
		{
			KxVetorizado[i] = vetFromMatlab[i];
		}		
	}
	
	if(!compareStringRef)
	{
		for(size_t i = 0; i < size_vetFromMatlab; i++)
		{
			RefVetorizado[i] = vetFromMatlab[i];
		}
	}
	
	if(!compareStringCoefRef)
	{
		for(size_t i = 0; i < size_vetFromMatlab; i++)
		{
			coefRefVetorizado[i] = vetFromMatlab[i];
		}
	}
	
	if(!compareStringSV)
	{
		for(size_t i = 0; i < size_vetFromMatlab; i++)
		{
			SVVetorizado[i] = vetFromMatlab[i];
		}
	}
	
	if(!compareStringTrim)
	{
		for(size_t i = 0; i < size_vetFromMatlab; i++)
		{
			trimVetorizado[i] = vetFromMatlab[i];
		}
	}

}

void LQR::imprimirMat(char flag, char qualMatriz)
{
	//imprimir de forma formatada
	
	if(flag == 'f')
	{
			if(qualMatriz == '1')
			{
				for(int i = 0; i < iK1; i++)
				{
					for(int j = 0; j < jK1; j++)
					{
						printf("%f ", K1[i][j]);
					}
					printf("\n");
				}
			}
		
			if(qualMatriz == 'x')
			{
				for(int i = 0; i < iKx; i++)
				{
					for(int j = 0; j < jKx; j++)
					{
						printf("%f ", Kx[i][j]);
					}
					printf("\n");
				}
			}
	}
	
	//imprime o indice seguido do valor
	
	else
	{
		if(qualMatriz == '1')
		{
			for(int j = 0; j < jK1; j++)
			{
				for(int i = 0; i < iK1; i++)
				{
					printf("[%d][%d] %f\n", i, j, K1[i][j]);
				}
			}
		}
		
		if(qualMatriz == 'x')
		{
			for(int j = 0; j < jKx; j++)
			{
				for(int i = 0; i < iKx; i++)
				{
					printf("[%d][%d] %f\n", i, j, Kx[i][j]);
				}
			}
		}
	}
	
	printf("\n");
}	

void LQR::computeControl(/*antradas: e1 ref, e2 sv*/)
{
    computeRef();    

    //SV são os estados que virão do matlab (XX)

    //*
    arma::mat errL; //Create local variable for erro
    
    errL = armaRef - armaSV(arma::span(9,11), 0); //Reference-actual
    errL = armaK1 * errL;	//First loop LQR
        
    armaError = armaError + errL * dt; //Integrate error
   
    armaOutputControl = armaKx * armaSV;	//Second loop LQR
    armaOutputControl = armaError - armaOutputControl + armaTrim;	//Actuator Output
    
    //para enviar os dados para o matlab o dado deve estar verorizado
    vetorizarOutControl();

}

void LQR::vetorizarOutControl()
{
	 //converter para vetor para enviar para o matlab
	  for(size_t i = 0; i < iOutputControl; i++)
		{
			for(size_t j = 0; j < jOutputControl; j++)
			{
				OutputControlVetorizado[i] = arma::as_scalar( armaOutputControl(i, j) );
			}
		}
}

void LQR::setArmaMatrix(arma::mat &ArmaMatrix, size_t sizeLin, size_t sizeCol)
{
		//seta iterativamente novos valores
		//-geralmente usada para setar valores de SV
		for(size_t j = 0; j < sizeCol; j++)
		{
			for(size_t i = 0; i < sizeLin; i++)
			{
				ArmaMatrix(i, j) = SV[i][j];
			}
		}
}

void LQR::setStateVariables(double *XXvetFromMatlab, size_t sizeColXXvetFromMatlab)
{
	recvMatrizVetorizada(XXvetFromMatlab, sizeColXXvetFromMatlab, "SV");
	
	constructMatrix(iSV, jSV, "SV");
	
	setArmaMatrix(armaSV, iSV, jSV);
	
}

void LQR::printVet(double *vet, size_t sizeVet)
{
	std::cout << "\n<< PRINT VET >>\n";
	for(size_t i = 0; i < sizeVet; i++)
	{
		std::cout << i << " " << vet[i] << std::endl;
	}
	std::cout << "\n";
}

void LQR::initReferencia(double *CoefRefVetMatlab)
{
	recvMatrizVetorizada(CoefRefVetMatlab, icoefRef*jcoefRef, "CoefRef");
	constructMatrix(icoefRef, jcoefRef, "CoefRef");
	
	//~ computeRef(); //Calcula a referencia inicial para caso ela seja fixa ou variavel
	
}

void LQR::init_dt(double dtMatlab)
{
	dt = dtMatlab;
}



void LQR::initTrim(double *trimVetMatlab)
{
	recvMatrizVetorizada(trimVetMatlab, iTrim*jTrim, "trim");
	
	constructMatrix(iTrim, jTrim, "trim");
	
}

void LQR::computeRef()
{
	//definição/
	t = t0 + dt;
	
	//declarar As e Bs de forma explicita por questao de clareza/legibilidade
	
	//x: Ax*t + Bx
	double Ax = coefRef[0][0];
	double Bx = coefRef[0][1];
	armaRef(0, 0) = Ax*t + Bx;
	
	//y: Ay*t + By
	double Ay = coefRef[1][0];
	double By = coefRef[1][1];
	armaRef(1, 0) = Ay*t + By;
	
	//z: Az*t + Bz
	double Az = coefRef[2][0];
	double Bz = coefRef[2][1];
	armaRef(2, 0) = Az*t + Bz;
	
	//atualização de t - pegar o ultimo valor para a proxima iteração
	t0 = t;
}

double* LQR::getOutputControl()
{
	return OutputControlVetorizado;
}

//~ int main()
//~ {

	//fonte: http://arma.sourceforge.net/docs.html#Mat
	
	/*
	For convenience the following typedefs have been defined:

    mat 	 =  	Mat<double>
    dmat 	 =  	Mat<double>
    fmat 	 =  	Mat<float>
    cx_mat 	 =  	Mat<cx_double>
    cx_dmat  =  	Mat<cx_double>
    cx_fmat  =  	Mat<cx_float>
    umat 	 =  	Mat<uword>
    imat 	 =  	Mat<sword>
    */
	
	/*
	// erro local - lqr
	arma::mat errolocal;
	// Parametro K1 utilizado no LQR
	arma::mat K1; 
    // Parametro Kx utilizado no LQR
    arma::mat Kx; 
    
    //setup de parametros K1 e Kx
    K1.set_size(4, 3);
    Kx.set_size(4, 12);
	
	//carregar de arquivo os parametros K1 e Kx
	
	
	//printando para debugar e testar
	std::cout << "errolocal: " << errolocal << std::endl;
	std::cout << "K1: " << K1 << std::endl;
	std::cout << "Kx: " << Kx << std::endl;	
	
	
	arma::mat A(5,10);  A.zeros(); 
	arma::mat B;

	B 	<< 1 << 3 << 5 << arma::endr
		<< 2 << 4 << 6 << arma::endr;
  
	std::cout << "Kx: " << B << std::endl;*/
	
	
	//~ LQR obj;
	//~ obj.printMatrix();
	
	//~ obj.constructMatrix(1,2,"oi");
	
	//~ obj.constructMatrix(1.1,2.2,"oi2");
	
	//~ obj.constructMatrix(1.1,2.2,"K1");
	//~ obj.constructMatrix(1.1,2.2,"Kx");
	//~ obj.constructMatrix(1.1,2.2,"Kakashi");
	
	//valores que serão recebidos do matlab
	//estes valores utilizados agora foram geados aleatoriamente (são simulados)
	//~ double 	K1VetMatlab[iK1*jK1] = {2, 3, 4, 5, 7, 9, 11, 12, 14, 18, 19, 20}, 
					//~ KxVetMatlab[iKx*jKx] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48};
	
	//~ obj.recvMatrizVetorizada(K1VetMatlab, KxVetMatlab, iK1*jK1, iKx*jKx);
	
	//~ printVet(obj.K1Vetorizado, 12);
	//~ printVet(obj.KxVetorizado, 48);
	
	//~ //----
	//~ obj.constructMatrix(iK1, jK1, "K1");	
	//~ obj.constructMatrix(iKx, jKx, "Kx");
	//~ //----
	
	//~ obj.imprimirMat('f', '1');
	//~ obj.imprimirMat('f', 'x');
	
	//~ LQR obj(K1VetMatlab, KxVetMatlab);
	//~ obj.imprimirMat('f', '1');
	//~ obj.imprimirMat('f', 'x');
	
	//~ return 0;
//~ }


