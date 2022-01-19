/*
 * control.cpp
 *
 *  Created on: Jul 8, 2020
 *      Author: -F.Machini
 */

#include "control.h"


Control::Control(int numStates, int numInput, double refVal, double timeSample){
        ref = refVal*3.1415/180;
        dt = timeSample;
	K.set_size(numInput,numStates);
        Kp.set_size(2,1);
	ifstream in_file;
	in_file.open("Kx.txt");
	if (!in_file)
	{
		std::cerr<< "File is not open"<<std::endl;
        }
	for (int j{0}; j<numStates; ++j){
	        for (int i{0}; i<numInput; ++i){
	        in_file>>K(i,j);
	    }
	};
        ifstream in_file2;
	in_file2.open("Kp.txt");
	if (!in_file)
	{
		std::cerr<< "File is not open"<<std::endl;
        }
	for (int j{0}; j<1; ++j){
	        for (int i{0}; i<2; ++i){
	        in_file2>>Kp(i,j);
	    }
	};
}

arma::mat Control::computeOutput(arma::mat States, int cont){
    mat Aux(2,1);
    err += (ref - States(cont,2))*dt;
    Aux(0,0)= States(cont,2);
    Aux(1,0) = States(cont,5);
    return Kp*err-K*Aux;
}

arma::mat Control::computeOutputLag(arma::mat States, int cont){
    mat out(2,1);
    ekm1 = ek;
    ek = -States(cont,2);
    wkm1 = wk;
    wk = 0.9999*wkm1 + 3.259*ek - 2.832*ekm1;
    out(0,0) = wk;
    out(1,0) = -wk;
    return out;
}

arma::mat Control::dutyLag(arma::mat LagDuty, int trim){
    mat duty(2,1);
    duty(0,0) = 1000000 + (LagDuty(0,0) + trim)*10000;
    duty(1,0) = 1000000 + (LagDuty(1,0) + trim)*10000;
    motorSaturation(duty);
    motorSaturation(duty);
    return duty;    
}


arma::mat Control::force2duty(arma::mat forceVec, double trimForce){
    //trimForce  = Force for equilibrium in Newtons (for each motor)
    mat duty(2,1);
    
    //duty(0,0) = 1000000 + ((forceVec(0,0) + 1.02 + trimForce)/0.0778)*10000; //values when using battery with period = 3000000
    //duty(1,0) = 1000000 + ((forceVec(1,0) + 1.02 + trimForce)/0.0778)*10000;
    duty(0,0) = 1000000 + ( (forceVec(0,0) + 2.2374 + trimForce)/0.0887)*10000;
    duty(1,0) = 1000000 + ( (forceVec(1,0) + 2.0675 + trimForce)/0.0822)*10000;
    
    motorSaturation(duty);
    
    return duty;
}

arma::mat Control::PWM2force(arma::mat PWM){
    
    mat force(2,1);
    force(0,0) = 0.0887*PWM(0,0)-2.2374;
    force(1,0) = 0.0822*PWM(1,0) - 2.0675;
            
    return force;
}

void Control::motorSaturation(mat &duty){
    
    for (int i{0}; i<2; ++i){
        
        if (duty(i,0) <1400000)
        { 
            duty(i,0) = 1400000;
        } else if (duty(i,0) > 1600000)
        {
            duty(i,0) = 1600000;
        } else 
        {       
            duty(i,0) = duty(i,0);
        }
    }
}



