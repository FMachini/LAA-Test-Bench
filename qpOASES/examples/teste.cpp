#include <bits/stdc++.h> //All basic library
#include <armadillo>
#include <qpOASES.hpp>

using namespace std;
using namespace arma;

int main(){
	USING_NAMESPACE_QPOASES

	/* Setup data of first QP. */
	real_t H[2*2] = { 1.0, 0.0, 0.0, 0.5 };
	real_t Ax[1*2] = { 1.0, 1.0 };
	real_t g[2] = { 1.5, 1.0 };
	real_t lb[2] = { 0.5, -2.0 };
	real_t ub[2] = { 5.0, 2.0 };
	real_t lbA[1] = { -1.0 };
	real_t ubA[1] = { 2.0 };

	/* Setup data of second QP. */
	real_t g_new[2] = { 1.0, 1.5 };
	real_t lb_new[2] = { 0.0, -1.0 };
	real_t ub_new[2] = { 5.0, -0.5 };
	real_t lbA_new[1] = { -2.0 };
	real_t ubA_new[1] = { 1.0 };


	/* Setting up QProblem object. */
	QProblem example( 2,1 );

	Options options;
	example.setOptions( options );

	/* Solve first QP. */
	int_t nWSR = 10;
	example.init( H,g,Ax,lb,ub,lbA,ubA, nWSR );

	/* Get and print solution of first QP. */
	real_t xOpt[2];
	real_t yOpt[2+1];
	example.getPrimalSolution( xOpt );
	example.getDualSolution( yOpt );
	printf( "\nxOpt = [ %e, %e ];  yOpt = [ %e, %e, %e ];  objVal = %e\n\n", 
			xOpt[0],xOpt[1],yOpt[0],yOpt[1],yOpt[2],example.getObjVal() );
	
	/* Solve second QP. */
	nWSR = 10;
	example.hotstart( g_new,lb_new,ub_new,lbA_new,ubA_new, nWSR );

	/* Get and print solution of second QP. */
	example.getPrimalSolution( xOpt );
	example.getDualSolution( yOpt );
	printf( "\nxOpt = [ %e, %e ];  yOpt = [ %e, %e, %e ];  objVal = %e\n\n", 
			xOpt[0],xOpt[1],yOpt[0],yOpt[1],yOpt[2],example.getObjVal() );

	example.printOptions();
	/*example.printProperties();*/

	/*getGlobalMessageHandler()->listAllMessages();*/
	//SS a tempo discreto
	mat A = {{0.9927,0.0194},
		{-0.7196,0.9413}};
	mat B = {{0.0024,-0.0023},
		 {0.2338,-0.2234}};
	mat C = {1,0};
	//auxiliares
	int n = A.n_rows;
	int p = B.n_cols;
	int q = C.n_rows;

	//Parametros de projeto
	int N = 5;
	int M = 3;
	mat rho = {1,1};
	mat mi = {1};
	rho = 7*rho;
	mi = 2*mi;
	mat Hqp,Aqp_du,Aqp_u,Aqp_y,Aqp,Gn,phi;
	cout<<"bug1"<<endl;
	arma::mat Ip(p,p,arma::fill::eye);
	cout<<"bug2"<<endl;	
	arma::mat Atil = join_rows(join_cols(A,arma::zeros(p,n)),join_cols(B,Ip));
	cout<<"bug3"<<endl;	
	arma::mat Btil = join_cols(B,Ip);
	cout<<"bug4"<<endl;	
	arma::mat Ctil = join_rows(C,arma::zeros(q,p));
	cout<<"bug5"<<endl;	
	arma::mat G(q*N,p*M,arma::fill::zeros);
	cout<<"bug6"<<endl;	
	for(int i = 0;i<N;i++){
		for(int j=0;j<std::min(i+1,M);j++){
			G.submat(i*q,j*p,(i+1)*q-1,(j+1)*p-1) = Ctil*arma::powmat(Atil,i-j)*Btil;
		}
	}
	cout<<"bug7"<<endl;
	mi = repmat(mi,1,N);
	cout<<"bug8"<<endl;	
	rho = repmat(rho,1,M);
	cout<<"bug9"<<endl;	
	arma::mat Qn = diagmat(mi);
	mat X =  {{2.0000,        0,        0,        0,        0},
        	  {0,   2.0000,        0,        0,        0},
        	  {0,       0,   2.0000,        0,        0},
        	  {0,        0,        0,   2.0000,        0},
        	  {0,        0,        0,        0,   2.0000}};

	cout<<"bug10"<<endl;	
	arma::mat Rm = diagmat(rho);
	cout<<"bug11"<<endl;
	//Hqp = 2*(G.t()*Qn*G+Rm);
	Hqp = G.t();
	cout<<"bug11.5"<<endl;
	cout<<G*G.t()<<endl;
	cout<<Hqp.size()<<endl;
	cout<<Qn.size()<<endl;
	Hqp = 2*(G.t()*Qn*G+Rm);
	cout<<"bug12"<<endl;
	//Gn = libMPC::matriz::Gn(mi,A,B,C,N,M,p,q,n);
	cout<<Hqp<<endl;
	cout<<"bug13"<<endl;	
	//cout<<Gn<<endl;
}
