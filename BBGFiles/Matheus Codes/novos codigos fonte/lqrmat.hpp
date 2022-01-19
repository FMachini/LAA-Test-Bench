#ifndef LQRMAT_HPP
#define LQRMAT_HPP

#include <iostream>
#include <armadillo>
#include <string.h>

#define iK1 4 	//linha
#define jK1 3 	//coluna
#define iKx 4 	//linha 
#define jKx 12 	//coluna
#define iRef 3
#define jRef 1  //ref vem do matlab como sendo os coeficiente A e b de x a z
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
 
//ref final tem que ser 3, 1 

class LQR
{
	public:
		double K1[iK1][jK1], Kx[iKx][jKx], Ref[iRef][jRef], SV[iSV][jSV], trim[iTrim][jTrim],
					 coefRef[icoefRef][jcoefRef];
		double K1Vetorizado[iK1*jK1], KxVetorizado[iKx*jKx], RefVetorizado[iRef*jRef], 
					 coefRefVetorizado[icoefRef*jcoefRef], SVVetorizado[iSV*jSV], trimVetorizado[iTrim*jTrim],
					 OutputControlVetorizado[iOutputControl*jOutputControl];
		arma::mat armaK1, armaKx;
		arma::mat armaRef; //Reference for controller
		arma::mat armaSV; //State variables
    		arma::mat armaOutputControl; //Control Variables
		arma::mat armaError; //Global error for control
		arma::mat armaTrim; //Inital Velocity for control
    		arma::mat armaCoefRef; //POR QUESTOES DE DEBUG
    		double dt; //Control time difference
    		double t0 = 0;
    		double t = 0;
    				
		LQR(double *K1VetMatlab, double *KxVetMatlab, double *CoefRefVetMatlab, double dtMatlab, double *trimVetMatlab);
		
		void initArmaMatrix();
		void initParameters(double *K1VetMatlab, double *KxVetMatlab, double *CoefRefVetMatlab, double dtMatlab, double *trimVetMatlab);
		void initMatrizesDeGanho(double *K1VetMatlab, double *KxVetMatlab);
		void printParameters();
		void constructMatrix(size_t linSize, size_t colSize, const char *matrix); //do matlab a matrix vem vetorizada
		void recvMatrizVetorizada(double *K1vetFromMatlab, double *K2vetFromMatlab, size_t size_K1vetFromMatlab, size_t size_KxvetFromMatlab);
		void recvMatrizVetorizada(double *vetFromMatlab, size_t size_vetFromMatlab, const char *paramControle);
		void imprimirMat(char flag, char qualMatriz);
		void computeControl();
		void setStateVariables(double *XXvetFromMatlab, size_t sizeColXXvetFromMatlab);
		void printVet(double *vet, size_t sizeVet); //para debugar
		void initReferencia(double *CoefRefVetMatlab);
		void init_dt(double dtMatlab);
		void setArmaMatrix(arma::mat &ArmaMatrix, size_t sizeLin, size_t sizeCol); //atribuir iterativamente valors para mat armaSV
		void initTrim(double *trimVetMatlab);
		void computeRef();
		double *getOutputControl();
		void vetorizarOutControl();
		
	//void setReference(...);
		
	private:
		//
};

#endif
