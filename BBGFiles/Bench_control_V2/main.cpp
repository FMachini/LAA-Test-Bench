#include <iostream>
#include <fstream>
#include <stdio.h>
#include <armadillo>
#include "getStates.h"
#include "motor.h"
#include "control.h"
#include <unistd.h>
//#include <bits/stdc++.h>

#define path_2_pwmchip "pwmchip0/"
#define export1 0 // Motor 1 export value (creates pwm0)
#define path_1_pwmchip "pwmchip1/"// motor address
#define export2 0 // Motor 1 export value (creates pwm1)



using namespace std;
using namespace arma;

int numit, numIT,disbalance;
double dt,trim;
int in{1};
std::string file2save;
std::vector<double> data;
std::vector<double> angle;
std::vector<double> enc0;
std::vector<double> enc;
std::vector<double> rateNoFilter; // output from low pass filter vector
double eptrV, eptrH,eV0,eH0,thetaRef;
clock_t t0, tend; 
int numStates{2},numInput{2};
mat Ctrout(1,1);
mat Duty(2,1);
mat trimPWM(2,1);
mat initTrim(2,1,fill::zeros);  


int main()
{
    // Getting system parameters from txt file
    ifstream sysParam;
    sysParam.open("System_parameters.txt");
    if (!sysParam)
	{
        std::cerr<< "File does not exist"<<std::endl;
    }
    sysParam>>dt; //time sample
    sysParam>>numit; //number of iterations
    sysParam>>numIT; // number of initalization
    sysParam>>trim; // Force value at hoover condition
    sysParam>> file2save; //file to save data name
    sysParam>> thetaRef;
    
    mat armaSV(numit,6,fill::zeros); //state matrix
    //vec rateData = zeros<vec>(3); //state vector
    //
    getData system(dt);
    Control Ctr(numStates,numInput,thetaRef,dt);
    system.imuInitialization();
    Motor motor1(path_1_pwmchip,export1); // creating motor 1 object
    Motor motor2(path_2_pwmchip,export2); // creating motor 2 object
    
    std::cout << "Hit 0 to set initial state"<< std::endl;
    
    while (in!=0)
    {
        cin>>in;
    }
    // initializing filter and motor
    int cont_while = 0;
     while (cont_while < 100)
    {
        t0 = clock();
        data = system.imuRawData(); // return accelerations, angular velocities an non-filtered angles
        angle = system.imuFilteredData(data); 
        cont_while++;    
        motor1.setDutyCycle(1155000);
        motor2.setDutyCycle(1155000);
        system.waitSample(t0);
    }
   
    // set trim duty-cycle
    trimPWM = Ctr.force2duty(initTrim,trim);;
    motor1.setDutyCycle(trimPWM(0,0));
    motor2.setDutyCycle(trimPWM(1,0));
    system.encoderInitialPosition(eH0, eV0); // getting encoder initial position
    
    eptrV = eV0; eptrH = eH0;  
    
    std::cout << "Bias Corrected"<< std::endl;
    
    ofstream  file;
    file.open(file2save);
       
    // Begin experiment
    
    std::cout << "Hit 1 to begin experiment"<< std::endl;
     while (in!=1)
    {
        cin>>in;
    }
    
    std::cout << "Collecting Data"<< std::endl;
    
    int cont_while2 = 0;
    
    while (cont_while2 < numit)
    {    

	t0 = clock();
	data = system.imuRawData(); // return accelerations, angular velocities an non-filtered angles
        
	angle = system.imuFilteredData(data); // return filtered roll and pitch angles and rates  x = [roll,roll_rate, pitch, pitch_rate];
        std::cout << angle[0] << " " << angle[1] << std::endl;        
        //enc = system.imuFilteredData(data);        
        enc = system.encoderPosition(eH0,eV0,eptrV,eptrH);  // return encoder angular position and rates x = [theta_1, theta_1_rate, theta_2, theta_2_rate];
        
       //std::cout << angle[0] << " " << angle[1]  << " "  << angle[2] << " " << angle[3] << " " << enc[0] << " " << enc[1] << " " << enc[2] << " " << enc[3] << std::endl;
        //std::cout << armaSV(0) << " " << armaSV(1)  << " "  << armaSV(2) << " " << armaSV(3) << " " << armaSV(4) << " " << armaSV(5) << std::endl; 
        system.getStateVector(angle,enc,armaSV,cont_while2);
        
        //system.lowPassFilter(angle,armaSV,rateNoFilter,cont_while2);
        // apply move average filter to velocity vector
        system.moveMean(armaSV,4,cont_while2);
        
        // Calculating Control Effort
        // Using LQR or control allocation
//        if (cont_while2 <50)
//        { 
//            disbalance = 1000000 + (trim+5)*10000;
//            motor1.setDutyCycle(disbalance);
//            motor2.setDutyCycle(trimPWM);
//            
//        } else{
        
        Ctrout = Ctr.computeOutput(armaSV,cont_while2);
        Duty = Ctr.force2duty(Ctrout,trim);
        
        //Using Lag controller
        //Ctrout = Ctr.computeOutputLag(armaSV,cont_while2);
        //Duty = Ctr.dutyLag(Ctrout,trim);
        
        //cout<< " Control Input "<<Ctrout(0,0)<< endl; 
        motor1.setDutyCycle(Duty(0,0));
        motor2.setDutyCycle(Duty(1,0));  
//            motor1.setDutyCycle(trimPWM);
 //           motor2.setDutyCycle(trimPWM);
        //cout<< " Control Input "<<Duty(0,0)<< endl; 
        //}
        //angle.push_back(armaSV(cont_while2,5)); //save filtered angular velocity
        //enc.push_back(armaSV(cont_while2,3)); // save encoder filtered angular velocity
        //enc.push_back(armaSV(cont_while2,4));
        enc.push_back(Ctrout(0,0));
      // Saving states @ file    	
        system.saveData(file,angle);
        system.saveData(file,enc);
        file << "\n";

       
//        tend   = clock(); 
//        cout<< "sample time "<<(tend-t0)/1000;      
        system.waitSample(t0);
//        tend   = clock(); 
//        cout<< " "<<(tend-t0)/1000<<endl; 
	cont_while2++;       
	   
    }
        cout<< " Size State Matrix "<<size(armaSV)<< endl; 
	file.close();	 
	std::cout << "Finished Data Acquisition"<< std::endl;
        
        // Deactivating ESC
        motor1.deactivateESC();
        motor2.deactivateESC();

    return 0; 
}
