/*
 * control.cpp
 *
 *  Created on: Jul 8, 2020
 *      Author: -F.Machini
 */

#include "control.h"


Control::Control(int numStates, int numInput, int numTrack, arma::mat refVal, double timeSample){
        ref.set_size(size(refVal));
        err.set_size(size(refVal)); err.zeros();
        ref = refVal*3.1415/180;
        std::cerr<< "Reference set to: Theta_1 "<< refVal(0,0) << " and Theta_2 "<< refVal(1,0)<< std::endl;
        dt = timeSample;
	K.set_size(numInput,numStates);
        Kp.set_size(numInput,numTrack);
	ifstream in_file;
	in_file.open("Kx.txt");
	if (!in_file)
	{
		std::cerr<< "File is not open"<<std::endl;
        }
	for (int i{0}; i<numInput; ++i){
	        for (int j{0}; j<numStates; ++j){
	        in_file>>K(i,j);
	    }
	};
        in_file.close();
        ifstream in_file2;
	in_file2.open("Kp.txt");
	if (!in_file2)
	{
		std::cerr<< "File is not open"<<std::endl;
        }
	for (int i{0}; i<numInput; ++i){
	        for (int j{0}; j<numTrack; ++j){
	        in_file2>>Kp(i,j);
	    }
	};
        in_file2.close();
        Kp.print();
        K.print();
}

arma::mat Control::computeOutput(arma::mat States, int cont){
    mat Aux2(2,1);
    mat Aux1 = trans(States(cont,span::all));
    
    Aux2(0,0)= States(cont,0);
    Aux2(1,0) = States(cont,1); 
   
    err += Kp*(ref - Aux2)*dt;

    return err-K*Aux1;
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


arma::mat Control::force2duty(arma::mat forceVec, int trim){
    double trimForce, minForce, maxForce;
    mat duty(2,1);
    trimForce = PWM2force(trim);

    duty(0,0) = 1000000 + ((forceVec(0,0) + 1.39 + trimForce)/0.0917)*10000;
    duty(1,0) = 1000000 + ((forceVec(1,0) + 1.39 + trimForce)/0.0917)*10000;

    motorSaturation(duty);
    motorSaturation(duty);
    return duty;
}

arma::mat Control::angle2duty(float angleVec){

    mat duty(2,1);
    
    duty(0,0) = (-18.27632*angleVec +  2.2799e+03)*1000; // Tilt 1
    duty(1,0)= (-18.27632*angleVec +  2.2799e+03)*1000; // Tilt 2

    % servoSaturation(duty);
    % servoSaturation(duty);
    return duty;
}


double Control::PWM2force(int PWM){
    
    return 0.0917*PWM - 1.39;
}

void Control::motorSaturation(mat &duty){
    
    for (int i{0}; i<2; ++i){
        
        if (duty(i,0) <1350000)
        { 
            duty(i,0) = 1350000;
        } else if (duty(i,0) > 1450000)
        {
            duty(i,0) = 1450000;
        } else 
        {       
            duty(i,0) = duty(i,0);
        }
    }
}



