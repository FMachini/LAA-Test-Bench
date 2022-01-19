#ifndef _SENSORS_H_
#define _SENSORS_H_

#include <iostream>
#include <iomanip>
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

class Sensors
{
private:
	MPU6050 mpu;
	Kalman kalman1;
	Kalman kalman2;
	int16_t accx, accy, accz, accx2, accy2, accz2, gyrox, gyroy, gyroz;
	float dt;
	eQEP eqepVertical{eQEP2, eQEP::eQEP_Mode_Absolute};
	eQEP eqepHorizontal{eQEP1, eQEP::eQEP_Mode_Absolute};
	double time_taken;
	clock_t tf;
	double ykm2{0}, ykm1{0}, xkm1{0}, xkm2{0}, HP0{0}, VP0{0}, eptrV{0}, eptrH{0}, eV0{0}, eH0{0};

	public:

	Sensors(float sampleTime);
	std::vector<double> imuOffset_correction(double accx, double accy, double accz, double gyrox, double gyroy, double gyroz);
	double imuRoll(double accx, double accy, double accz);
	double imuPitch(double accx, double accy, double accz);
	std::vector<double> imuRawData();
	void imuInitialization();
	std::vector<double> imuFilteredData(std::vector<double> rawData);

	std::vector<double> encoderPosition();

	void encoderInitialPosition();
	void getStateVector(std::vector<double> fromImu, std::vector<double> fromEncoder, arma::mat &SV,int numit);
	void saveData(ofstream &file,std::vector<double> data2save);
	void waitSample(double t1);
	double lowPassFilter(double xk);
	void moveMean(arma::mat &SV, const int k, int nit);
	void printData(const arma::mat data);

};




#endif
