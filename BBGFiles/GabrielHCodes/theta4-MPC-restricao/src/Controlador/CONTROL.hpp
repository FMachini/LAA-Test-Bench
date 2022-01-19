#ifndef CONTROL_HPP
#define CONTROL_HPP

#include <fstream>
#include <armadillo>
#include <vector>
#include <iostream>
#include <string>

#include <qpOASES.hpp>
#include "include/libMPC.h"

#define rad2deg (180/3.14159265358979323846)
#define deg2rad (3.14159265358979323846/180)
using namespace qpOASES;
using namespace std;
using namespace arma;

class Control{
	private:
		mat u0;
		mat ref;

		//Auxiliares
		int n,p,q,nb,pb,qb;
		mat Ip,Ipb;
		//Horizontes
		int N,M;
		//Modelo Linear
		mat A,B,C;
		//Modelo Linear considerando pertubacao
		mat Ab,Bb,Cb;
		//Ganho LQR do MPCmiLQR
		mat K_mpc;
		//Ganho LQR do observador
		mat LL_mpc;
		//Pesos MPC
		mat rho,mi;
		//Variaveis MPC
		mat Pdu,Pu,PI,Hdu,Hu,Hqp,Aqp,Gn,Phi;
		//Coef Forca
		mat Fcoef1,Fcoef2;
		//Limites
		mat uMaxN,uMinN,duMaxN,duMinN;
		//Referencia MPC
		mat r;
		//auxiliares
		mat ukm1,umpckm1,chiKm1Km1,duMpc,uMpc,uLqr,u;
		//problema de otimizacao
		SQProblem mpc;
		//auxiliares qpoases
		int_t nWSR;
	public:
		Control(int numStates, int numInput, int numRef, arma::mat refVal, double timeSample, double forces_trim[2]);
		void initializeQproblem(int p, int N, int M);
		void setRefTheta4inGraus(double referencia);
		arma::mat getRefinGraus();
		arma::mat INPUT(mat States,mat OUTPUT);
};

#endif
