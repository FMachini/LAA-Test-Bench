#include "include/libMPC.h"
#include "CONTROL.hpp"
#include <armadillo>
#include <qpOASES.hpp>
using namespace qpOASES;
using namespace arma;

Control::Control(int numStates, int numInput, int numRef, arma::mat refVal, double timeSample, double forces_trim[2]){
	//**********************************************************
	//INICIALIZACAO
	u0 << forces_trim[0] << endr
       << forces_trim[1] << endr;

	ref.set_size(size(refVal));
	ref = refVal;
	A.set_size(numStates,numStates);
	B.set_size(numStates,numInput);
	C.set_size(numRef,numStates);
	K_mpc.set_size(numInput,numStates);
	LL_mpc.set_size(numRef+numStates,numRef);
	mi.set_size(numRef);
	rho.set_size(numInput);
	
	//**********************************************************
	ifstream in_file;
	in_file.open("A.txt");
	if (!in_file)
	{
		std::cerr<< "File A is not open"<<std::endl;
	}
	for (int i{0}; i<numStates; ++i){
		for (int j{0}; j<numStates; ++j){
			in_file>>A(i,j);
		}
	};
	in_file.close();
	cout <<"Matriz A: "<< endl << A << endl;

	//**********************************************************
	in_file.open("B.txt");
	if (!in_file)
	{
		std::cerr<< "File B is not open"<<std::endl;
	}
	for (int i{0}; i<numStates; ++i){
		for (int j{0}; j<numInput; ++j){
			in_file>>B(i,j);
		}
	};
	in_file.close();
	cout <<"Matriz B: "<< endl << B << endl;

	//**********************************************************
	in_file.open("C.txt");
	if (!in_file)
	{
		std::cerr<< "File C is not open"<<std::endl;
	}
	for (int i{0}; i<numRef; ++i){
		for (int j{0}; j<numStates; ++j){
			in_file>>C(i,j);
		}
	};
	in_file.close();
	cout <<"Matriz C: "<< endl << C << endl;

	//**********************************************************
	n = A.n_rows;
	p = B.n_cols;
	q = C.n_rows;

	//**********************************************************
	in_file.open("K_mpc.txt");
	if (!in_file)
	{
		std::cerr<< "File K_mpc is not open"<<std::endl;
	}
	for (int i{0}; i<numInput; ++i){
		for (int j{0}; j<numStates; ++j){
			in_file>>K_mpc(i,j);
		}
	};
	in_file.close();

	cout <<"Matriz K: "<< endl << K_mpc << endl;

	//**********************************************************
	in_file.open("LL_mpc.txt");
	if (!in_file)
	{
		std::cerr<< "File LL_mpc is not open"<<std::endl;
	}
	for (int i{0}; i<(numRef+numStates); ++i){
		for (int j{0}; j<numRef; ++j){
			in_file>>LL_mpc(i,j);
		}
	};
	in_file.close();
	cout <<"Matriz LL: "<< endl << LL_mpc << endl;

	//**********************************************************
	in_file.open("mu.txt");
	if (!in_file)
	{
		std::cerr<< "File mu is not open"<<std::endl;
	}
	for (int i{0}; i<numRef; ++i){
			in_file>>mi(i);
	};
	in_file.close();
	mi = mi.t();
	cout <<"Matriz mi: "<< endl << mi << endl;

	//**********************************************************
	in_file.open("rho.txt");
	if (!in_file)
	{
		std::cerr<< "File rho is not open"<<std::endl;
	}
	for (int i{0}; i<numInput; ++i){
		in_file>>rho(i);
	};
	in_file.close();
	rho=rho.t();
	cout <<"Matriz rho: "<< endl << rho << endl;

	//**********************************************************
	in_file.open("horizontes.txt");
	if (!in_file)
	{
		std::cerr<< "File horizontes is not open"<<std::endl;
	}
	for (int i{0}; i<2; ++i){
		if(i==0){
			in_file>>N;
		}
		else{
			in_file>>M;
		}
	};
	in_file.close();
	cout <<"Horizontes: "<< endl << N << "\t" << M << endl;

	//**********************************************************
	initializeQproblem(p,N,M);

	//Definindo variaveis utilizadas no otimizador
	paramPlanta MATRIZES(K_mpc,A,B,C,rho,mi,N,M);
	Pdu = MATRIZES.matrizPdu();
	Pu = MATRIZES.matrizPu();
	PI = MATRIZES.matrizPI();
	Hdu = MATRIZES.matrizHdu();
	Hu = MATRIZES.matrizHu();
	Hqp = MATRIZES.matrizHqp();
	Aqp = MATRIZES.matrizAqp();	
	Gn = MATRIZES.matrizGn();	
	Phi = MATRIZES.matrizPhi();	
	Ab = MATRIZES.matrizAb();	
	Bb = MATRIZES.matrizBb();	
	Cb = MATRIZES.matrizCb();

	//**********************************************************
	nb = Ab.n_rows;
	pb = Bb.n_cols;
	qb = Cb.n_rows;

	//**********************************************************
	//Definindo restricoes
	mat uMax = {2.9 - u0(0),2.9 - u0(1)};
	mat uMin = {1.3 - u0(0),1.3 - u0(1)};
	uMax = uMax.t();
	uMin = uMin.t();
	mat duMax = {0.8,0.8};
	duMax = duMax.t();
	mat duMin = -duMax;
	
	uMaxN = repmat(uMax,N,1);
	uMinN = repmat(uMin,N,1);
	duMaxN = repmat(duMax,N,1);
	duMinN = repmat(duMin,N,1);

	//**********************************************************
	//Referencia
	r = repmat<mat>(ref,N,1);
	//INICIALIZACAO QPOSASES
	mat xINI = zeros<mat>(n,1);
	ukm1 = zeros<mat>(p,1);
	umpckm1 = zeros<mat>(p,1);
	chiKm1Km1 = zeros<mat>(nb,1);
	duMpc = zeros<mat>(p,1);	
	mat csi = join_cols(chiKm1Km1,ukm1);

	//**********************************************************
	//Calcular Fs
	mat f = Phi*csi;
	mat fqp = 2 * Gn.t() * (f - r);
	mat fu = Pu * xINI;
	mat fdu = Pdu * xINI;
	mat fI = PI * ukm1;
	mat umpckm1N = repmat(umpckm1,N,1);
	
	//**********************************************************
//	// Calcular Bqp
	mat bqpDU = join_cols((duMaxN - fdu + fI - Hdu*umpckm1N),(-duMinN + fdu - fI + Hdu*umpckm1N));
	mat bqpU = join_cols((uMaxN - Hu*umpckm1N - fu),(-uMinN + Hu*umpckm1N + fu));
	mat bqp = join_cols(bqpDU,bqpU);
	
	//**********************************************************
	// QPOASES configuracoes
	Options myOptions;
	//myOptions.setToDefault();
	myOptions.setToMPC();
	myOptions.printLevel = PL_LOW;

	mpc.setOptions( myOptions );

	// Working set
	int_t nWSR = 5*(Hqp.n_rows + Aqp.n_rows);

	// Auxiliares qpOASES
	real_t Hqp_aux[Hqp.n_elem];
	real_t Aqp_aux[Aqp.n_elem];
	real_t fqp_aux[Hqp.n_rows];
	real_t bqp_aux[Aqp.n_rows];
	real_t dutil[p*M];


	// arma::mat para real_t (array)
	for(int i=0;i<(Hqp.n_rows);i++){
		fqp_aux[i] = fqp(i);
		for(int j=0;j<Hqp.n_cols;j++){
			Hqp_aux[j+(i*Hqp.n_cols)]=Hqp(i,j);
			}
		}

	for(int i=0;i<(Aqp.n_rows);i++){
		bqp_aux[i] = bqp(i);
		for(int j=0;j<Aqp.n_cols;j++){
			Aqp_aux[j+(i*Aqp.n_cols)]=Aqp(i,j);
			}
		}
	mpc.init(Hqp_aux,fqp_aux,Aqp_aux,0,0,0,bqp_aux,nWSR);
	cout <<"INICIALIZACAO DO CONTROLE FINALIZADA"<<endl;
//**********************************************************
//**********************************************************
}


void Control::initializeQproblem(int p, int N,int M){
	SQProblem arg(p*M,4*p*N);
	mpc = arg;
}


mat Control::INPUT(arma::mat States,mat OUTPUT){
	//**********************************************************
	//Estados
	mat x(n,1);
	x = States;
	mat y(q,1);
	y = OUTPUT;
	
	//**********************************************************
	//Estimativa do estado chi
	mat chiKKm1 = Ab * chiKm1Km1 + Bb * umpckm1;
	mat yKKm1 = Cb * chiKKm1;
	mat chiKK = chiKKm1 + LL_mpc * (y - yKKm1);
	chiKm1Km1 = chiKK;
	
	//**********************************************************	
	// Estado artificial
	mat csi = join_cols(chiKK,umpckm1);
	
	//**********************************************************
	//Calcular Fs
	mat f = Phi*csi;
	mat fqp = 2 * Gn.t() * (f - r);
	mat fu = Pu * x;
	mat fdu = Pdu * x;
	mat fI = PI * ukm1;
	mat umpckm1N = repmat(umpckm1,N,1);
	
	//**********************************************************
	// Calcular Bqp
	mat bqpDU = join_cols((duMaxN - fdu + fI - Hdu*umpckm1N),(-duMinN + fdu - fI + Hdu*umpckm1N));
	mat bqpU = join_cols((uMaxN - Hu*umpckm1N - fu),(-uMinN + Hu*umpckm1N + fu));
	mat bqp = join_cols(bqpDU,bqpU);
	// arma::mat para real_t (array)
	real_t Hqp_aux[Hqp.n_elem];
	real_t Aqp_aux[Aqp.n_elem];
	real_t fqp_aux[Hqp.n_rows];
	real_t bqp_aux[Aqp.n_rows];
	real_t dutil[p*M];		
	// arma::mat para real_t (array)
	for(int i=0;i<(Hqp.n_rows);i++){
		fqp_aux[i] = fqp(i);
		for(int j=0;j<Hqp.n_cols;j++){
			Hqp_aux[j+(i*Hqp.n_cols)]=Hqp(i,j);
			}
		}

	for(int i=0;i<(Aqp.n_rows);i++){
		bqp_aux[i] = bqp(i);
		for(int j=0;j<Aqp.n_cols;j++){
			Aqp_aux[j+(i*Aqp.n_cols)]=Aqp(i,j);
			}
		}
	//**********************************************************
	// Calcular predicao do incremento na acao de controle
	nWSR = 5*(Hqp.n_rows + Aqp.n_rows);
	mpc.hotstart(Hqp_aux,fqp_aux,Aqp_aux,0,0,0,bqp_aux,nWSR);
	mpc.getPrimalSolution(dutil);
	// Definir o incremento na acao de controle a ser aplicado	
	for(int i = 0;i<p;i++){
		duMpc(i) = dutil[i];
	}

	//**********************************************************
	//Atualizar o controle aplicado
	uMpc = umpckm1 + duMpc;
	uLqr = -K_mpc * x;

	u = uLqr + uMpc;

	ukm1 = u;
	umpckm1 = uMpc;   
	return u + u0;
}
	
void Control::setRefTheta4inGraus(double referencia){
	ref =referencia*deg2rad;
	r = repmat<mat>(ref,N,1);
}
arma::mat Control::getRefinGraus(){
	mat erre = {r(0)};
	return(erre*rad2deg);
}


