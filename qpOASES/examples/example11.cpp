#include <iostream>
#include <armadillo>
#include <qpOASES.hpp>
#include "libMPC.h"
using namespace std;
using namespace arma;
using namespace qpOASES;


int main() {
		real_t Hxx[2*2] = {1,0,0,0.5};
		real_t Axx[1*2] = { 1.0, 1.0 };
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
		example.init(Hxx,g,Axx,0,0,0,0,nWSR);

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
		arma_rng::set_seed_random();

		cout << "Hello w44orld com armadillo!Fácil fácil" << endl;
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
		Hqp = libMPC::matriz::Hqp(rho,mi,A,B,C,N,M,p,q,n);
		cout<<Hqp<<endl;			
		return 0;
}
