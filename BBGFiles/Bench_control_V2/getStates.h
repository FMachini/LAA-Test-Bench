#ifndef _GET_STATE_H_
#define _GET_STATE_H_

#include <iostream>
#include <cstdint>
#include <string>

// IMU headers
#include <cmath>
#include <chrono> 
#include "MPU6050/MPU6050.h"
#include "Kalman/Kalman.h"
#include <vector>
#include <unistd.h>
#include <armadillo>

// encoder files
#include "eqep.h"
#include <math.h>
#include <fstream>


#define RAD2DEG 180.0/3.1415
#define DEG2RAD 3.1415/180
// Class 
using namespace std;

class getData
{
	private:
		MPU6050 mpu;
		Kalman kalman1;
                Kalman kalman2;		
		int16_t accx, accy, accz, accx2, accy2, accz2, gyrox, gyroy, gyroz;
		float dt;
		//std::string eQEP3 = "/sys/devices/platform/ocp/48304000.epwmss/48304180.eqep";
		eQEP eqepVertical{eQEP2, eQEP::eQEP_Mode_Absolute};
                eQEP eqepHorizontal{eQEP1, eQEP::eQEP_Mode_Absolute};
                double time_taken; 
                clock_t tf;
                double ykm2{0}, ykm1{0}, xkm1{0}, xkm2{0};
		// double eptr{V;double eptrH;	
			
	public:
            
		getData(float sampleTime);
		std::vector<double> imuOffset_correction(double accx, double accy, double accz, double gyrox, double gyroy, double gyroz);
		double imuRoll(double accx, double accy, double accz);
		double imuPitch(double accx, double accy, double accz);
		std::vector<double> imuRawData();
		void imuInitialization();
		std::vector<double> imuFilteredData(std::vector<double> rawData);
		
		std::vector<double> encoderPosition(const double &HP0,const double &VP0,double &eptrV,double &eptrH);
                
                void encoderInitialPosition(double &eH0, double &eV0);
                void getStateVector(std::vector<double> fromImu, std::vector<double> fromEncoder, arma::mat &SV,int numit);
                void saveData(ofstream &file,std::vector<double> data2save);
                void waitSample(double t1);
                //void lowPassFilter(std::vector<double> collectData,arma::mat &y, std::vector<double> &x,int nit);
                double lowPassFilter(double xk);
                void moveMean(arma::mat &SV, const int k, int nit);

};




#endif