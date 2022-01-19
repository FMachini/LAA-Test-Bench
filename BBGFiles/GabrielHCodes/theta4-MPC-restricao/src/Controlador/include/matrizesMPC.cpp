#include "libMPC.h"
#include <armadillo>

using namespace arma;

paramPlanta::paramPlanta(mat Kd,mat Ad,mat Bd, mat Cd, mat rhod, mat mid, int Nd, int Md){
	//Modelo linear	
	A = Ad;
	B = Bd;
	C = Cd;
	//Ganho LQR
	K = Kd;
	//Peso MPC
	rho = rhod;
	mi = mid;
	//Horizontes
	N = Nd;
	M = Md;
	//Auxiliares
	n = A.n_rows;	// n. Estados
        p = B.n_cols;	// n. Entradas
        q = C.n_rows;	// n. Saidas
	Ip = eye<mat>(p,p);
	Iq = eye<mat>(q,q);
	//Modelo considerando LQR na malha interna
	Ak = A - B * K;
	
	//Modelo considernado perturbacao
	Ab = join_cols(join_rows(Ak,arma::zeros(n,q)),join_rows(arma::zeros(q,n),Iq));
	Bb = join_cols(B,arma::zeros(q,p));
	Cb = join_rows(C,Iq);
	
	//Auxiliares
	nb = Ab.n_rows;	// n. Estados
        pb = Bb.n_cols;	// n. Entradas
        qb = Cb.n_rows;	// n. Saidas
	Ipb = eye<mat>(pb,pb);

	//Modelo em termos de DU
	Atil = join_rows(join_cols(Ab,arma::zeros(pb,nb)),join_cols(Bb,Ipb));
	Btil = join_cols(Bb,Ipb);
	Ctil = join_rows(Cb,arma::zeros(qb,pb));
	
	//Variaveis da equacao de predicao dos estados [x]
	Px = Ak;
	for(int i = 1;i<N;i++){
		Px=join_cols(Px,arma::powmat(Ak,i+1));
	}
	Hx = zeros<mat>(n*N,p*N);
	for(int i = 0;i<N;i++){
		for(int j=0;j<std::min(i+1,N);j++){
			Hx.submat(i*n,j*p,(i+1)*n-1,(j+1)*p-1) = arma::powmat(Ak,i-j)*B;
		}
	}
	//Variaveis da equacao de predicao do ulqr
	Plqr = join_cols(-K,zeros<mat>(p*(N-1),n));
	Hlqr = zeros<mat>(p*N,n*N);
	for(int i=1;i<N;i++){
		int j = i - 1;
		Hlqr.submat(i*p,j*n,(i+1)*p-1,(j+1)*n-1) = -K;
	}
	//Variaveis da equacao de predicao do controle [u] em termos de uMpc
	Pu = Plqr + Hlqr * Px;
	Hu = Hlqr*Hx + eye<mat>(p*N,p*N);
	//Variaveis da equacao de predicao das variaveis controladas [y] em termos
	//de duMpc
	phi = Ctil*Atil;
	for(int i = 1;i<N;i++){
		phi=join_cols(phi,Ctil*arma::powmat(Atil,i+1));
	}
	G = zeros<mat>(qb*N,pb*M);
	for(int i = 0;i<N;i++){
		for(int j=0;j<std::min(i+1,M);j++){
			G.submat(i*qb,j*pb,(i+1)*qb-1,(j+1)*pb-1) = Ctil*arma::powmat(Atil,i-j)*Btil;
		}
	}
	//Variaveis da equacao de predicao do controle [u] em termos do duMpc
	mat row(1,p*M,fill::zeros); row(0) = 1;
	mat col(p,1,fill::zeros); col(0) = 1;
	col = repmat(col,N,1);
	mat T_NxM_Ip = toeplitz(col,row);
	HT = Hu * T_NxM_Ip;
	//Variaveis da equacao de predicao do incremento de controle total [du] em
	//termos do duMpc
	HI = eye<mat>(p*N,p*N);
	for(int i=1;i<N;i++){
		int j = i - 1;
		HI.submat(i*p,j*p,(i+1)*p-1,(j+1)*p-1) = -eye<mat>(p,p);
	}
	PI = join_cols(eye<mat>(p,p),zeros<mat>(p*N-p,p));
	Pdu = HI * Pu;
	Hdu = HI * Hu;
	HIT = HI*HT;
	//Variaveis do problema de programacao quadratica
	mi = repmat(mi,1,N);
	rho = repmat(rho,1,M);
	Qn = diagmat(mi);
	Rm = diagmat(rho);
	Gn = Qn*G;
	Hqp = 2*(G.t() * Qn * G + Rm);
	
	mat restDu = join_cols(HIT,-HIT);
	mat restU = join_cols(HT,-HT);
		
	Aqp = join_cols(restDu,restU);	
}
mat paramPlanta::matrizPdu(){
	return Pdu;
}

mat paramPlanta::matrizPu(){
	return Pu;
}

mat paramPlanta::matrizPI(){
	return PI;
}

mat paramPlanta::matrizHdu(){
	return Hdu;
}

mat paramPlanta::matrizHu(){
	return Hu;
}

mat paramPlanta::matrizHqp(){
	return Hqp;
}

mat paramPlanta::matrizAqp(){
	return Aqp;
}

mat paramPlanta::matrizGn(){
	return Gn;
}

mat paramPlanta::matrizPhi(){
	return phi;
}
mat paramPlanta::matrizAb(){
	return Ab;
}

mat paramPlanta::matrizBb(){
	return Bb;
}

mat paramPlanta::matrizCb(){
	return Cb;
}