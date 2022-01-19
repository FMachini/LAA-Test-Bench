#include <iostream>
#include <fstream>
#include <stdio.h>
// #include <armadillo>
#include "getStates.h"
#include "motor.h"
#include <unistd.h>
//#include <bits/stdc++.h>

#define path_1 "pwmchip3/pwm0/" // Motor 1 address
#define path_2 "pwmchip1/pwm1/" // Motor 2 address
#define dt 0.02 //sampling time in seconds

using namespace std;
// using namespace arma;

std::vector<double> data;
std::vector<double> angle;
std::vector<double> enc0;
std::vector<double> enc;
double time_taken; 
clock_t t0, tf; 

 // Function to wait sample time during simulation

void waitSample(double t1){
    time_taken = 0;
        
        while (time_taken < dt*1000000){
           tf = clock();
           time_taken = (tf-t1);
         }   
}

int main()
{
    getData system(0.02);
    system.imuInitialization();
    Motor motor1(path_1); // creating motor 1 object
    Motor motor2(path_2); // creating motor 2 object
    
    
    int cont_while = 0;
     while (cont_while < 100)
    {
        //t0 = clock();
        data = system.imuRawData(); // return accelerations, angular velocities an non-filtered angles
        angle = system.imuFilteredData(data); 
        cont_while++;    
        time_taken = 0;
        
        //waitSample(t0);
    }
   
    
    enc0 = system.encoderInitialPosition(); // getting encoder initial position
    
      
    std::cout << "Bias Corrected"<< std::endl;
    //ofstream  file;
    //file.open("States_init_cond.txt");
    std::cout << "Collecting Data"<< std::endl;
    
    int cont_while2 = 0;
    
    while (cont_while2 < 500)
    {    
	//t0 = clock();
	data = system.imuRawData(); // return accelerations, angular velocities an non-filtered angles
        
	angle = system.imuFilteredData(data); // return filtered roll and pitch angles and rates  x = [roll,roll_rate, pitch, pitch_rate];
        
        t0 = clock();
        //enc = system.imuFilteredData(data);        
        enc = system.encoderPosition(enc0);  // return encoder angular position and rates x = [theta_1, theta_1_rate, theta_2, theta_2_rate];
        tf = clock();
        cout << "Time taken by program is : " << (tf - t0)<< endl; 
        motor1.setDutyCycle(1200000);
        motor2.setDutyCycle(1200000);
        std::cout << angle[0] << " " << angle[1]  << " "  << angle[2] << " " << angle[3] << " " << enc[0] << " " << enc[1] << " " << enc[2] << " " << enc[3] << std::endl;
    // Saving states @ States.txt file     
                
        //waitSample(t0);
	cont_while2++;       
	   
    }
	//file.close();	
	std::cout << "Finished Data Aquisition"<< std::endl;

    	
        
     // mat A(2,3);  // directly specify the matrix size (elements are uninitialised)
  
    //cout << "A.n_rows: " << A.n_rows << endl;  // .n_rows and .n_cols are read only
    //cout << "A.n_cols: " << A.n_cols << endl;
    return 0; 
}
