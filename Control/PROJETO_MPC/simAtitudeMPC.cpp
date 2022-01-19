//**********************************************************
// Bibliotecas
#include <bits/stdc++.h>//All basic libraries
#include <armadillo>	//std::arma
#include <boost/numeric/odeint.hpp>
#include <qpOASES.hpp>
#include "libMPC.h"

//**********************************************************		
using namespace arma;
using namespace std;
using namespace boost::numeric::odeint;
using namespace qpOASES;

//**********************************************************
//Variaveis Globais
#define pi 3.14159265

//**********************************************************		
//Dinamica da atitude do bicoptero
class fnlin{
	mat u;
public:
	fnlin(mat param) : u(param) {}
	void operator()(const vector<double> &x, vector<double> &dxdt, const double t){
		double	a = 0.0187;
		double	b = 0.0495;
		double	c = 0.6925;
		double	p = 5.348;
		double	q = 13.43;
		double	r = 1.319;
		double	L1 = 0.225;
		double	L2 = 0.215;

		dxdt[0] = x[3];
		dxdt[1] = x[4];
		dxdt[2] = x[5];
		dxdt[3] = (1/a)*(L1*x[1] - L2*x[2] - b*x[3] - c*x[0]);
		dxdt[4] = r*u(0) - p*x[4] - q*x[1];
		dxdt[5] = r*u(1) - p*x[5] - q*x[2];
	}
};

//**********************************************************
// Rotina principal		
int main(){
	//**********************************************************
	//SS a tempo discreto
	mat A = {{0.9927,0.0194},
                {-0.7196,0.9413}};
        mat B = {{0.0024,-0.0023},
                 {0.2338,-0.2234}};
        mat C = {1,0};

	//**********************************************************
	//Auxiliares
	int n = A.n_rows;	// n. Estados
        int p = B.n_cols;	// n. Entradas
        int q = C.n_rows;	// n. Saidas

	//**********************************************************
	//Parametros do controlador
	double posIni = 0;		// Posicao inicial
	mat yref = {25*pi/180};		// Referencia
 
	double Ts = 0.02;	// Periodo de amostragem (segundos)
	double Td = 0.01, dt = Td/40;	// Atraso de transporte
	
	int N = 5;	// Horizonte de predicao
        int M = 3;	// Horizonte de controle
        mat rho = {1,1};	// Peso das entradas
        mat mi = {1};		// Peso das saidas
        rho = 7*rho;
        mi = 2*mi;

	mat wMax; wMax << 10 << endr << 10 << endr;	//Limitante superior de PWM
	mat wMin; wMin <<-10 << endr <<-10 << endr;	//Limitante inferior de PWM	

	mat uMax = 1.319/13.43 * wMax;	//Limitante superior de forca
	mat uMin = 1.319/13.43 * wMin;	//Limitante superior de forca

	mat duMax; duMax << 5.2 << endr << 5.2 << endr; //Limitante superior da variacao de forca por periodo
	mat duMin; duMin << -4 << endr << -4 << endr;	//LImitante inferior da variacao de forca por periodo
	duMax = duMax*0.8*2/9;
	duMin = duMin*0.8*2/9;

	//**********************************************************
	// Matrizes de predicao
	mat Hqp,Aqp_du,Aqp_u,Aqp,Gn,phi;
        Hqp = libMPC::matriz::Hqp(rho,mi,A,B,C,N,M,p,q,n);
        Gn = libMPC::matriz::Gn(mi,A,B,C,N,M,p,q,n);
        Aqp_du = libMPC::matriz::Aqp_du(p,M);
        Aqp_u = libMPC::matriz::Aqp_u(p,M);
        Aqp = join_cols(Aqp_du,Aqp_u);
        phi = libMPC::matriz::phi(A,B,C,N,p,q,n);

	//**********************************************************
	//Variaveis de estado e controle
	int kmax = 400;
	// Estado x
	n=n+4;
	mat x = zeros<mat>(n,kmax+1);
	// Entrada
	mat u = zeros<mat>(p,kmax);
	mat w = zeros<mat>(p,kmax);
	// Incremento da entrada
	mat du = zeros<mat>(p,kmax);
	// Saida
	mat y = zeros<mat>(q,kmax);
	mat ym = zeros<mat>(q,kmax);
	//CPU time
	mat cpu_time = zeros<mat>(kmax,1);
	clock_t tictoc;

	//**********************************************************
	//Condicoes iniciais
	vector<double> xini(6);
	xini[0] = posIni;
	x(0,0) = xini[0];
	mat wkm1 = zeros<mat>(p,1);
	mat ukm1 = zeros<mat>(p,1);
	mat wAux, xAux;
	xAux << x(0,0) << endr << x(3,0) << endr;

	//**********************************************************
	//Referencia
	mat r = repmat<mat>(yref,N,1);
	
	//**********************************************************
	//Ruido posicao angular - Range de 0.05 rad
	//Ruido velocidade angular - Range de 0.4 rad/s
	mat ruido(2,kmax);
	for(int i = 0; i<kmax;i++){
		ruido(0,i) = ((double)((rand()%5001)-2500))/((double)100000);
		ruido(1,i) = ((double)((rand()%28001)-14000))/((double)100000);
	}
	
	//**********************************************************
	// Estado artificial
	mat csi;
	csi = join_cols((xAux + ruido.col(0)),ukm1);

	//**********************************************************
	// Calcular f
	mat f, fqp;
	f = phi*csi;
	fqp = 2 * Gn.t() * (f - r);
	
	//**********************************************************
	// Calcular Bqp
	mat bqpDU, bqpU, bqp;
	bqpDU = join_cols(repmat(duMax,M,1),repmat(duMin,M,1));
	bqpU = join_cols(repmat((uMax - ukm1),M,1),repmat((uMin - ukm1),M,1));
	bqp = join_cols(bqpDU,bqpU);
	
	//**********************************************************
	// QPOASES configuracoes
	QProblem mpc(p*M,p*M);
	Options myOptions;
	myOptions.setToMPC();
	myOptions.printLevel = PL_LOW;
	mpc.setOptions( myOptions );

	//**********************************************************
	// Working set
	int_t nWSR = 5*(Hqp.n_rows + Aqp.n_rows);

	//**********************************************************
	// Auxiliares qpOASES
	real_t Hqp_aux[Hqp.n_elem];
	real_t Aqp_aux[Aqp.n_elem];
	real_t fqp_aux[Hqp.n_rows];
	real_t bqp_aux[Aqp.n_rows];
	real_t dutil[p*M];

	//**********************************************************
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
	mpc.init(Hqp_aux,fqp_aux,Aqp_aux,0,0,0,bqp_aux,nWSR);
	//mpc.getPrimalSolution(dutil);

	//**********************************************************
	// Evolucao da dinamica da planta
	for(int k = 0; k<kmax;k++){
		tictoc = clock();
		xAux << x(0,k) << endr << x(3,k) << endr;

		//**********************************************************
		// Saida Real
		y.col(k) = C * xAux;

		//**********************************************************
		// Saida medida
		ym.col(k) = C * xAux + ruido(0,k);

		//**********************************************************
		// Estado artificial
		csi = join_cols((xAux + ruido.col(k)),ukm1);

		//**********************************************************
		// Calcular f
		f = phi*csi;
		fqp = 2 * Gn.t() * (f - r);
		
		//**********************************************************
		// Calcular Bqp
		bqpU = join_cols(repmat((uMax - ukm1),M,1),repmat((uMin - ukm1),M,1));
		bqp = join_cols(bqpDU,bqpU);

		//**********************************************************
		// arma::mat para real_t (array)		
		for(int i=0;i<Hqp.n_rows;i++){
			fqp_aux[i] = fqp(i);
		}
		for(int i=0;i<Aqp.n_rows;i++){
			bqp_aux[i] = bqp(i);
		}
	
		//**********************************************************
		// Calcular predicao do incremento na acao de controle
		nWSR = 5*(Hqp.n_rows + Aqp.n_rows);
		mpc.hotstart(fqp_aux,0,0,0,bqp_aux,nWSR);
		mpc.getPrimalSolution(dutil);
		
		//**********************************************************
		// Definir o incremento na acao de controle a ser aplicado
		for(int i = 0;i<p;i++){
			du(i,k) = dutil[i];
		}

		//**********************************************************
		// Atualizar o controle aplicado
		u.col(k) = ukm1 + du.col(k);
		
		//**********************************************************
		// Converter u (forca) para w (PWM)
		w.col(k) = 13.43/1.319 * u.col(k);
		
		//**********************************************************
		tictoc = clock()-tictoc;		
		cpu_time(k) = ((float)tictoc)/CLOCKS_PER_SEC*1000000;
		
		//**********************************************************
		// Simulacao da dinamica da planta
		for(int j = 0;j < n;j++){xini[j] = x(j,k);}
		// 0 - Td
		fnlin fnlinTd(wkm1);
		integrate(fnlinTd,xini,0.0,Td,dt);
		//Td - Ts
		fnlin fnlinTs(w.col(k));
		integrate(fnlinTs,xini,0.0,Ts-Td,dt);

		for(int z = 0;z < n;z++){
			x(z,k+1) = xini[z];
		}

		//**********************************************************
		wkm1 = w.col(k);
		ukm1 = u.col(k);
	}

	//**********************************************************
	//Armazenamento dos dados
	y = y.t() * 180/pi;
	ym = ym.t() * 180/pi;
	u = u.t();
	w = w.t();
	du = du.t();
	x = x.cols(0,kmax-1).t();

	mat k = linspace<mat>(0,kmax-1,kmax);
	mat data;

	data = join_rows(k,y);
	data = join_rows(data,ym);
	data = join_rows(data,w);
	data = join_rows(data,x);
	data = join_rows(data,u);
	data = join_rows(data,du);
	data = join_rows(data,cpu_time);
	data.save("data.txt", raw_ascii);

	//**********************************************************
	//Definindo cpu time medio para calcular a acao de controle
	mat time = mean(cpu_time,0);
	mat dtime = stddev(cpu_time,0);
	cout<<"CPU time: "<<time(0)<<"+/-"<<2*dtime(0)<<" microsegundos, para uma confianca de 95%."<<endl;

	return 0;
}
