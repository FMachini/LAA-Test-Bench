/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   control.h
 * Author: -F.Machini
 *
 * Created on July 3, 2020, 1:36 PM
 */

#ifndef CONTROL_H
#define CONTROL_H
#include <fstream>
#include <armadillo>
#include <vector>

using namespace std;
using namespace arma;

class control {
public:
    control(int numStates, int numCtrl);
    arma computeControl(arma SV);
    vector<int> force2pwm(arma force);
private:
    arma K;
};

#endif /* CONTROL_H */

