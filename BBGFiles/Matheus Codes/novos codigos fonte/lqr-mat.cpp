#include <iostream>
#include <armadillo>

//fonte: http://arma.sourceforge.net/docs.html#Mat


class LQR
{
	public:
	double K1[1][2];
	double Kx[1][1];
	double a1 = K1[0][0], a2 = K1[0][1];
	
	
	
	arma::mat B;

	B 	<< 1 << 3 << 5 << arma::endr
		<< 2 << 4 << 6 << arma::endr;
  
	


};

int main()
{
	/*
	For convenience the following typedefs have been defined:

    mat 	 =  	Mat<double>
    dmat 	 =  	Mat<double>
    fmat 	 =  	Mat<float>
    cx_mat 	 =  	Mat<cx_double>
    cx_dmat  =  	Mat<cx_double>
    cx_fmat  =  	Mat<cx_float>
    umat 	 =  	Mat<uword>
    imat 	 =  	Mat<sword>
    */
	
	// erro local - lqr
	arma::mat errolocal;
	// Parametro K1 utilizado no LQR
	arma::mat K1; 
    // Parametro Kx utilizado no LQR
    arma::mat Kx; 
    
    //setup de parametros K1 e Kx
    K1.set_size(4, 3);
    Kx.set_size(4, 12);
	
	//carregar de arquivo os parametros K1 e Kx
	
	
	//printando para debugar e testar
	std::cout << "errolocal: " << errolocal << std::endl;
	std::cout << "K1: " << K1 << std::endl;
	std::cout << "Kx: " << Kx << std::endl;	
	
	
	arma::mat A(5,10);  A.zeros(); 
	/*arma::mat B;

	B 	<< 1 << 3 << 5 << arma::endr
		<< 2 << 4 << 6 << arma::endr;
  
	std::cout << "Kx: " << B << std::endl;*/
	
	return 0;
}
