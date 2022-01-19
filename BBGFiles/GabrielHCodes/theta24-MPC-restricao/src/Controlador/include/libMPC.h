#pragma once
#include <armadillo>

using namespace std;
using namespace arma;

class paramPlanta{
	private:
		//Auxiliares
		int n,p,q,nb,pb,qb;
		mat Ip,Iq,Ipb;
		//Horizontes
		int N,M;
		//Modelo Linear
		mat A,B,C;
		//Modelo Linear expandido considerando pertubacao
		mat Ab,Bb,Cb;
		//Ganho LQR
		mat K;
		//Pesos MPC
		mat rho,mi;

		//Modelo artifical
		mat Ak, Atil,Btil,Ctil;
		//Variaveis da equacao de predicao dos estados [x]
		mat Px,Hx;
		//Variaveis da equacao de predicao do ulqr
		mat Plqr,Hlqr;
		//Variaveis da equacao de predicao do controle [u] em termos de uMpc
		mat Pu,Hu;
		//Variaveis da equacao de predicao das variaveis controladas [y] em termos
		//de duMpc
		mat phi,G;
		//Variaveis da equacao de predicao do controle [u] em termos do duMpc
		mat HT;
		//Variaveis da equacao de predicao do incremento de controle total [du] em
		//termos do duMpc
		mat HI,PI,Pdu,Hdu,HIT;
		//Variaveis do problema de programacao quadratica
		mat Qn,Rm,Gn,Hqp,Aqp;
	public:	
		paramPlanta(mat Kd,mat Ad,mat Bd,mat Cd, mat rhod, mat mid, int Nd, int Md);
		mat matrizPdu();
		mat matrizPu();
		mat matrizPI();
		mat matrizHdu();
		mat matrizHu();
		mat matrizHqp();
		mat matrizAqp();
		mat matrizGn();
		mat matrizPhi();
		mat matrizAb();
		mat matrizBb();
		mat matrizCb();
	};

