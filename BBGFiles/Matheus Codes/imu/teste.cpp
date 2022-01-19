#include <iostream>
#include <fstream>
#include <stdio.h>
//#include <chrono>
//#include <thread>//f
#include <cmath>

#include "MPU6050/MPU6050.h"
#include "Kalman/Kalman.h"
using namespace std;

#define RAD_TO_DEG 180.0/3.1415

double roll(double accx, double accy, double accz);

int main()
{
    // MPU INITIALIZATION
    MPU6050 mpu;
    mpu.initialize();
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
    Kalman kalman;

    //Load parameters
    int accel_offsets[3] = {-3037, -428, 2155};
    int gyro_offsets[3] = {58, 1, -39};

    mpu.setXAccelOffset(accel_offsets[0]);
    mpu.setYAccelOffset(accel_offsets[1]);
    mpu.setZAccelOffset(accel_offsets[2]);

    mpu.setXGyroOffset(gyro_offsets[0]);
    mpu.setYGyroOffset(gyro_offsets[1]);
    mpu.setZGyroOffset(gyro_offsets[2]);

    int16_t accx, accy, accz, gyrox, gyroy, gyroz;
	
	double roll, pitch;

    //-mpu.getMotion6(&accx, &accy, &accz, &gyrox, &gyroy, &gyroz);
    //-kalman.setAngle(roll(accx, accy, accz))

    ofstream  file;
    int cont_while = 0;
    file.open("Dados_teste.txt");
    while (cont_while < 3000)
    {
        mpu.getMotion6(&accx, &accy, &accz, &gyrox, &gyroy, &gyroz);

        std::cout << "Valores puros do imu:" << std::endl;
        std::cout << "accx: " << accx << std::endl;
        std::cout << "accy: " << accy << std::endl;
        std::cout << "accz: " << accz << std::endl;
        std::cout << "gyrox: " << gyrox << std::endl;
        std::cout << "gyroy: " << gyroy << std::endl;
        std::cout << "gyroz: " << gyroz << std::endl;
        std::cout << "===========================" << std::endl;
	
	roll = atan2(accy,(sqrt(accx*accx + accz*accz)))*180.0/3.1415;
	pitch = atan2((-accx),(sqrt(accy*accy + accz*accz)))*180.0/3.1415;

	std::cout << "roll: " << roll << std::endl;
	std::cout << "pitch: " << pitch << std::endl; 
	
	if (file.is_open())
	{	
		file << accx << " " << accy << " " << accz << " " << gyrox << " " << gyroy << " " << gyroz << " " << roll << " " << pitch << "\n";
	}
	cont_while++;

   }
    file.close();
    return 0;
}

double roll(double accx, double accy, double accz)
{
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  return (atan2(accY, accZ) * RAD_TO_DEG);
#else
  return (atan(accy / sqrt(accx * accx + accz * accz)) * RAD_TO_DEG);
#endif
}
