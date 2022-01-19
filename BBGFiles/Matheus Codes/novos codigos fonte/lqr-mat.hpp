#ifndef LQR-MAT_H
#define LQR-MAT_H

#include <iostream>
#include <armadillo>
#include <string.h>

#define iK1 4 	//linha
#define jK1 3 	//coluna
#define iKx 4 	//linha 
#define jKx 12 	//coluna

class LQR
{
	public:
		double K1[iK1][jK1];
		double Kx[iKx][jKx];
		double K1Vetorizado[iK1*jK1], KxVetorizado[iKx*jKx];
		arma::mat B;
		
		LQR(double *K1VetMatlab, double *KxVetMatlab);
		
		void initMatrix(); //testando so
		void initParameters(double *K1VetMatlab, double *KxVetMatlab);
		void initMatrizesDeGanho(double *K1VetMatlab, double *KxVetMatlab);
		void printMatrix();
		void constructMatrix(size_t linSize, size_t colSize, const char *matrix); //do matlab a matrix vem vetorizada
		void recvMatrizVetorizada(double *K1vetFromMatlab, double *K2vetFromMatlab, size_t size_K1vetFromMatlab, size_t size_KxvetFromMatlab);
		void imprimirMat(char flag, char qualMatriz);
		
	private:

};

#endif
