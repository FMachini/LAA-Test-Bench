#include <iostream>
#include <fstream>
#include <stdio.h>
//#include <chrono>
//#include <thread>//f
#include <cmath>
#include <chrono> 
#include "MPU6050/MPU6050.h"
#include "Kalman/Kalman.h"
#include <vector>
#include <unistd.h>
using namespace std;
using namespace std::chrono;

#define RAD_TO_DEG 180.0/3.1415

double getRoll(double accx, double accy, double accz);
double getPitch(double accx, double accy, double accz);
std::vector<double> offset_correction(double accx, double accy, double accz, double gyrox, double gyroy, double gyroz);

int main()
{
    // MPU INITIALIZATION
    MPU6050 mpu;
    mpu.initialize();
    mpu.setDLPFMode(MPU6050_DHPF_1P25);
    
    
    if (mpu.testConnection())
    {
        std::cout << "MPU connection successful!" << std::endl;
    }
    else
    {
        std::cout << "MPU connection failed!" << std::endl;
        return -1;
    }

    //Instanciar filtro
    Kalman kalman1;
    Kalman kalman2;

    //Load parameters
    //int accel_offsets[3] = {-4096/2, -4096/2, -2048/2};
    //int gyro_offsets[3] = {-178, -27, -35};
    float roll_cor;
	float pitch_cor;
	float dt;
	std::vector<double> out(6);
	
    //mpu.setXAccelOffset(accel_offsets[0]);
    //mpu.setYAccelOffset(accel_offsets[1]);
    //mpu.setZAccelOffset(accel_offsets[2]);

    //mpu.setXGyroOffset(gyro_offsets[0]);
    //mpu.setYGyroOffset(gyro_offsets[1]);
    //mpu.setZGyroOffset(gyro_offsets[2]);

    int16_t accx, accy, accz, accx2, accy2, accz2, gyrox, gyroy, gyroz;
	double acx, acy, acz, gx, gy, gz;
	
	double roll, pitch;

    mpu.getMotion6(&accx, &accy, &accz, &gyrox, &gyroy, &gyroz);
    
    out = offset_correction(accx, accy, accz, gyrox, gyroy, gyroz);
    
    kalman1.setAngle(getRoll(out[0], out[1], out[2]));
    kalman2.setAngle(getPitch(out[0], out[1], out[2]));

    ofstream  file;
    int cont_while = 0;
    file.open("Dados_teste.txt");
    while (cont_while < 1000)
    {
    //auto start = high_resolution_clock::now();
    
        mpu.getMotion6(&accx, &accy, &accz, &gyrox, &gyroy, &gyroz);
		mpu.getAcceleration(&accx2, &accy2, &accz2);
		
		out = offset_correction(accx, accy, accz, gyrox, gyroy, gyroz);
		
		acx = out[0];
		acy = out[1];
		acz = out[2];
		gx =  out[3];
		gy = out[4];
		gz = out[5];
		
        std::cout << "Valores puros do imu:" << std::endl;
        std::cout << "accx: " << accx << "    acx: " << acx << std::endl;
        std::cout << "accy: " << accy << "    acy: " << acy << std::endl;
        std::cout << "accz: " << accz << "    acz: " << acz <<  std::endl;
        std::cout << "gyrox: " << gyrox << std::endl;
        std::cout << "gyroy: " << gyroy << std::endl;
        std::cout << "gyroz: " << gyroz << std::endl;
        std::cout << "===========================" << std::endl;
	
	//roll = atan2(accy,(sqrt(accx*accx + accz*accz)))*180.0/3.1415;
		
	roll = getRoll(acx, acy, acz);//atan2(acy,(sqrt(acx*acx + acz*acz)))*180.0/3.1415;
	pitch = getPitch(acx, acy, acz);//atan2((-acx),(sqrt(acy*acy + acz*acz)))*180.0/3.1415;
	
	//auto stop = high_resolution_clock::now();
	//auto duration = duration_cast<seconds>(stop - start);
	usleep(50000);
	dt = 0.05;//duration.count();
	
	roll_cor = kalman1.getAngle(roll, gx, dt);
	pitch_cor = kalman2.getAngle(pitch, gy, dt);
	
	std::cout << "roll: " << roll << std::endl;
	std::cout << "pitch: " << pitch << std::endl;
	std::cout << "roll Kalman: " << roll_cor << std::endl;
	std::cout << "pitch Kalman: " << pitch_cor << std::endl; 	 
	
	if (file.is_open())
	{	
		file << accx << " " << accy << " " << accz << " " << gyrox << " " << gyroy << " " << gyroz << " " << roll << " " << pitch << roll_cor << " " << pitch_cor <<  "\n";
	}
	cont_while++;

   }
    file.close();
    return 0;
}

double getRoll(double accx, double accy, double accz)
{
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  return (atan2(accY, accZ) * RAD_TO_DEG);
#else
  return (atan(accy / sqrt(accx * accx + accz * accz)) * RAD_TO_DEG);
#endif
}

double getPitch(double accx, double accy, double accz)
{
	return (atan2((-accx),(sqrt(accy*accy + accz*accz)))*RAD_TO_DEG);
}

std::vector<double> offset_correction(double accx, double accy, double accz, double gyrox, double gyroy, double gyroz)
{
	std::vector<double> aux(6);
	    aux[0] = accx - 12196;
		aux[1] = accy - 7100;
		aux[2] = accz + 20594;
		aux[3] = (gyrox - 79)/131;
		aux[4] = (gyroy - 1466)/131;
		aux[5] = (gyroz - 7868)/131;
		
		return aux;
		
}
