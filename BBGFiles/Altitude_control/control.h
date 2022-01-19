/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   control.h
 * Author: -F.Machini
 *
 * Created on July 20, 2020, 2:47 PM
 */

#ifndef CONTROL_H
#define CONTROL_H

#include <iostream>
#include <string>
#include <armadillo>
#include <fstream>
#define force2dutyRelation 1000000;

using namespace std;
using namespace arma;

class Control{

	private:
	mat K, Kp, ref;
        mat err; 
        double ekm1, ek, wkm1, wk, dt;
        
	public:
	Control(int numStates, int numInput, int numTrack, arma::mat refVal, double timeSample);
	arma::mat computeOutput(arma::mat States,int cont);
	arma::mat force2duty(arma::mat forceVec, int trim);
        double PWM2force(int PWM);
        void motorSaturation(mat &duty);
        arma::mat computeOutputLag(arma::mat States, int cont);
        arma::mat dutyLag(arma::mat LagDuty, int trim);
};

#endif /* CONTROL_H */
