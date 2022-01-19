#pragma once
#include <armadillo>

namespace libMPC{
	class matriz{
	public:
		static arma::mat Hqp(arma::mat rho,arma::mat mi,arma::mat A,arma::mat B,arma::mat C,int N, int M, int p, int q, int n);
		static arma::mat Aqp_du(int p, int M);
		static arma::mat Aqp_u(int p, int M);
		static arma::mat Aqp_y(arma::mat A,arma::mat B,arma::mat C,int N, int M, int p, int q, int n);
		static arma::mat Gn(arma::mat mi,arma::mat A,arma::mat B,arma::mat C,int N, int M, int p, int q, int n);
		static arma::mat phi(arma::mat A,arma::mat B,arma::mat C,int N, int p, int q, int n);
	};
}
