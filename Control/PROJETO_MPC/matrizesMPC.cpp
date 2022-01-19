#include "libMPC.h"
#include <armadillo>

namespace libMPC{
	arma::mat matriz::Hqp(arma::mat rho,arma::mat mi,arma::mat A,arma::mat B,arma::mat C,int N,int M,int p,int q,int n){
		arma::mat Ip(p,p,arma::fill::eye);
		arma::mat Atil = join_rows(join_cols(A,arma::zeros(p,n)),join_cols(B,Ip));
		arma::mat Btil = join_cols(B,Ip);
		arma::mat Ctil = join_rows(C,arma::zeros(q,p));
		arma::mat G(q*N,p*M,arma::fill::zeros);
		for(int i = 0;i<N;i++){
			for(int j=0;j<std::min(i+1,M);j++){
				G.submat(i*q,j*p,(i+1)*q-1,(j+1)*p-1) = Ctil*arma::powmat(Atil,i-j)*Btil;
			}
		}
		mi = repmat(mi,1,N);
		rho = repmat(rho,1,M);
		arma::mat Qn = diagmat(mi);
		arma::mat Rm = diagmat(rho);
		return 2*(G.t()*Qn*G+Rm);
	}
	arma::mat matriz::Aqp_du(int p,int M){
		arma::mat IpM(p*M,p*M,arma::fill::eye);
		IpM = join_cols(IpM,-IpM);
		return IpM;
	}
	arma::mat matriz::Aqp_u(int p,int M){
		arma::mat row(1,p*M,arma::fill::zeros); row(0) = 1;
		arma::mat col(p,1,arma::fill::zeros); col(0) = 1;
		col = repmat(col,M,1);
		arma::mat TmIp = toeplitz(col,row);
		TmIp = join_cols(TmIp,-TmIp);
		return TmIp;
	}
	arma::mat matriz::Aqp_y(arma::mat A,arma::mat B,arma::mat C,int N,int M,int p,int q,int n){
		arma::mat Ip(p,p,arma::fill::eye);
		arma::mat Atil = join_rows(join_cols(A,arma::zeros(p,n)),join_cols(B,Ip));
		arma::mat Btil = join_cols(B,Ip);
		arma::mat Ctil = join_rows(C,arma::zeros(q,p));
		arma::mat G(q*N,p*M,arma::fill::zeros);
		for(int i = 0;i<N;i++){
			for(int j=0;j<std::min(i+1,M);j++){
				G.submat(i*q,j*p,(i+1)*q-1,(j+1)*p-1) = Ctil*arma::powmat(Atil,i-j)*Btil;
			}
		}
		G = join_cols(G,-G);
		return G;
	}
	arma::mat matriz::Gn(arma::mat mi,arma::mat A,arma::mat B,arma::mat C,int N,int M,int p,int q,int n){
		arma::mat Ip(p,p,arma::fill::eye);
		arma::mat Atil = join_rows(join_cols(A,arma::zeros(p,n)),join_cols(B,Ip));
		arma::mat Btil = join_cols(B,Ip);
		arma::mat Ctil = join_rows(C,arma::zeros(q,p));
		arma::mat G(q*N,p*M,arma::fill::zeros);
		for(int i = 0;i<N;i++){
			for(int j=0;j<std::min(i+1,M);j++){
				G.submat(i*q,j*p,(i+1)*q-1,(j+1)*p-1) = Ctil*arma::powmat(Atil,i-j)*Btil;
			}
		}
		mi = repmat(mi,1,N);
		arma::mat Qn = diagmat(mi);
		return Qn*G;
	}
	arma::mat matriz::phi(arma::mat A,arma::mat B,arma::mat C,int N,int p,int q,int n){
		arma::mat Ip(p,p,arma::fill::eye);
		arma::mat Atil = join_rows(join_cols(A,arma::zeros(p,n)),join_cols(B,Ip));
		arma::mat Ctil = join_rows(C,arma::zeros(q,p));
		arma::mat phi = Ctil*Atil;
		for(int i = 1;i<N;i++){
			phi=join_cols(phi,Ctil*arma::powmat(Atil,i+1));
		}
		return phi;
	}
}
