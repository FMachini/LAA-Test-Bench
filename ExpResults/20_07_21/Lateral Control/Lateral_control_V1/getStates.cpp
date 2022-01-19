#include <iostream>
#include <fstream>
#include <stdio.h>
//#include <chrono>
//#include <thread>//f
#include <cmath>
#include "MPU6050/MPU6050.h"
#include "Kalman/Kalman.h"
#include "eqep.h"
#include <vector>
#include <unistd.h>
using namespace std;
#include "getStates.h"


getData::getData(float sampleTime){
	
    mpu.initialize();  
    
    if (mpu.testConnection())
    {
        std::cout << "MPU connection successful!" << std::endl;
    }
    else
    {
        std::cout << "MPU connection failed!" << std::endl;
    }
    // set sampling time for Kalman filter
    dt = sampleTime;
    eqepHorizontal.set_period(200000L);
    eqepVertical.set_period(200000L);
	
}


double getData::imuRoll(double accx, double accy, double accz)
{
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  //roll =  (atan2(accY, accZ));	
  return (atan2(accY, accZ));
#else
  //roll = (atan(accy / sqrt(accx * accx + accz * accz)));
  return (atan(accy / sqrt(accx * accx + accz * accz))*RAD2DEG);
#endif
}

double getData::imuPitch(double accx, double accy, double accz)
{
	//pitch = (atan2((-accx),(sqrt(accy*accy + accz*accz))));
	return (atan2((-accx),(sqrt(accy*accy + accz*accz)))*RAD2DEG);
}

std::vector<double> getData::imuOffset_correction(double accx, double accy, double accz, double gyrox, double gyroy, double gyroz)
{
	//correcting Bias error based on experimental data
		std::vector<double> aux;
		aux.push_back(accx/16384*9.81);
		aux.push_back(accy/16384*9.81);
		aux.push_back((16384 - accz)/16384*9.81);
		aux.push_back(gyrox/131);
		aux.push_back(gyroy/131);
		aux.push_back(gyroz/131);
		return aux;		
}

void getData::imuInitialization(){
    // MPU INITIALIZATION
	std::vector<double> x0;  
	double roll, pitch;
	  
    std::cout << "Initializing!"<< std::endl;
    mpu.getMotion6(&accx, &accy, &accz, &gyrox, &gyroy, &gyroz);
	x0 = imuOffset_correction(accx, accy, accz, gyrox, gyroy, gyroz);
	roll = imuRoll(x0[0], x0[1], x0[2]);
        pitch = imuPitch(x0[0], x0[1], x0[2]);
    
    kalman1.setAngle(roll);
    kalman2.setAngle(pitch);
	
   std::cout << "Initial state set to " << roll << " and " << pitch << std::endl;
    
}


std::vector<double> getData::imuRawData()
{
    std::vector<double> raw;
    mpu.getMotion6(&accx, &accy, &accz, &gyrox, &gyroy, &gyroz);
    raw = imuOffset_correction(accx, accy, accz, gyrox, gyroy, gyroz);
    raw.at(4)=lowPassFilter(raw[4]);
    raw.push_back(imuRoll(raw[0], raw[1], raw[2]));
    raw.push_back(imuPitch(raw[0], raw[1], raw[2]));
	 
    return raw;
}

std::vector<double> getData::imuFilteredData(std::vector<double> rawData)
{
        
	std::vector<double> filterData;	
	filterData.push_back(kalman1.getAngle(rawData[6], rawData[3], dt)); // get roll angle        
        filterData.push_back(kalman1.getRate()); // get roll rate
	filterData.push_back(kalman2.getAngle(rawData[7], rawData[4], dt)); //get pitch angle
        filterData.push_back(kalman2.getRate()); // get pitch rate
        //filterData.push_back(rawData[6]);
	
	return filterData;
}

//void getData::lowPassFilter(std::vector<double> collectData,arma::mat &y, std::vector<double> &x,int nit)
//{
//    x.push_back(collectData[1]);
//    
//    // filter function
//    //y(k) = −a(2)y(k − 1) − a(3)y(k − 2) + b(1)x(k) + b(2)x(k − 1) + b(3)x(k + 2)
//    if (nit>2){
//    y(nit,5) = -0.2212 * y(nit-1,5) -0.1802 * y(nit-2,5)  + 0.3503 * x[nit] + 0.7007 * x[nit-1] + 0.3503 * x[nit-2];
//    }
// 
//}

double getData::lowPassFilter(double xk)
{
    double yk;
        yk = -0.2212 * ykm1 -0.1802 * ykm2  + 0.3503 * xk + 0.7007 * xkm1 + 0.3503 * xkm2;
        ykm2 = ykm1;
        ykm1=yk;
        xkm2=xkm1;
        xkm1=xk;
   
    return yk;
}

void getData::moveMean(arma::mat &SM, const int k, int nit)
{
    double sum1,sum2,sum3; 
    if (nit>k){         
         for (int j{0}; j<k; ++j)
         {
            sum1 += SM(nit-j,3);
            sum2 += SM(nit-j,4);
            sum3 += SM(nit-j,5);
         }
     }
            SM(nit,3) = sum1/k;
            SM(nit,4) = sum2/k;
            SM(nit,5) = sum3/k;
}

// encoder methods


void getData::encoderInitialPosition()
{
	 // std::vector<double> ex0;
	 //ex0.push_back(eqepVertical.get_position());
         //ex0.push_back(eqepHorizontal.get_position());
//         *eptrV = ex0[0];
//         *eptrH = ex0[1];
    eV0 = eqepVertical.get_position();
    eH0 = eqepHorizontal.get_position();
    eptrV = eV0; eptrH = eH0;

}

std::vector<double> getData::encoderPosition()
{
            std::vector<double> eout;
            double eVertical_Ang, eHorizontal_Ang,eVertical_Vel,eHorizontal_Vel;
	    // eVertical_p1 = eqepVertical.get_position();
            eVertical_Ang = ((eqepVertical.get_position()-eV0)*360/4000);
            eVertical_Ang  *= -1;             
            eVertical_Vel = (eVertical_Ang  - eptrV)/dt;
            eptrV = eVertical_Ang; 
             
            
	    // eHorizontal_p1= eqepHorizontal.get_position();
            eHorizontal_Ang= ((eqepHorizontal.get_position()-eH0)*360/4000);
	    eHorizontal_Ang *= -1;            
	    eHorizontal_Vel = (eHorizontal_Ang - eptrH)/dt;
            eptrH = eHorizontal_Ang; 
	                
            eout.push_back(eVertical_Ang);eout.push_back(eVertical_Vel);
            eout.push_back(eHorizontal_Ang);eout.push_back(eHorizontal_Vel);
            
	    return eout;
}

// Rearrange states vector to x = [theta_2,theta_1,theta_4] (already filtered!)
void getData::getStateVector(std::vector<double> fromImu, std::vector<double> fromEncoder, arma::mat &SV,int numit)
{
    SV(numit,0) = -fromEncoder[2]*DEG2RAD; //Theta_1 Reverse direction
    SV(numit,1) = fromEncoder[0]*DEG2RAD;    //Theta_2
    SV(numit,2) = fromImu[0]*DEG2RAD;   //Theta_4
    SV(numit,3) = -fromEncoder[3]*DEG2RAD; //Theta_1_dot Reverse direction
    SV(numit,4) = fromEncoder[1]*DEG2RAD; //Theta_2_dot
    SV(numit,5) = fromImu[1]*DEG2RAD; //Theta_4_dot

}

void getData::saveData(ofstream &saveFile,std::vector<double> data2save)
{
    if (!saveFile)
	{
        std::cerr<< "File is not open "<<std::endl;
    }
        for (int i{0}; i<data2save.size(); ++i)
        {
	saveFile << data2save[i] << " "; 
        };
 }


 // Function to wait sample time during simulation
void getData::waitSample(double t1)
{
    time_taken = 0;
    
        while (time_taken < dt*1000000){
           tf = clock();
           time_taken = (tf-t1);
         }   
}

void getData::printData(const arma::mat data,const int it){
    for (int i{0}; i<data.n_cols; ++i){
    std::cout<< data(it,i)*RAD2DEG << " ";    
    }
    std::cout<<endl;
}
