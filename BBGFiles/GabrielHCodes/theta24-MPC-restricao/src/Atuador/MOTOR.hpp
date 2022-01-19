#ifndef MOTOR_H
#define MOTOR_H

#include <iostream>
#include <fstream>
#include <stdio.h>
#include <string>
#include <math.h>
using namespace std;
#define rad2deg (180/3.1415)
#define deg2rad (3.1415/180)

class Motor{

private:
	float a {0.0008};
	float b {0.0090};
	float c {-0.2854};

	float forcePorcentMax = 60;      //In % of force total
	float forcePorcentMin = 40;      //In % of force total

	float forceNewtonMax = 2.95;     //In N
	float forceNewtonMin = 1.25;     //In N

	//float forceNewtonMax = 3.10;     //In N para o motor1
	//float forceNewtonMin = 1.28;     //In N para o motor1

	//float forceNewtonMax = 2.88;     //In N para o motor2
	//float forceNewtonMin = 1.25;     //In N para o motor2

	float trimForceinPorcent = 55;     //In % of force total
	float trimForceinNewton  = 2.22;   //In N

	float initialForceinPorcent = 15;  //In % of force total
	float initialForceinNewton  = 0.28;//In N

	string export_path={" "};
	string period_path={" "};
	string enable_path={" "};
	string duty_cycle_path={" "};

	string exp {"export"};
	string enable {"enable"};
	string period {"period"};
	string pwm_number {"pwm0/"};
	string pwmchip_number {"pwmchip0/"};
	string duty_cycle {"duty_cycle"};
	string root {"/sys/class/pwm/"};

	void setExport(int num_pwm);
	void setPeriod(int period);
	void setEnable(int enable);

    void setDutyCycle(int duty);
	void setValue(int val, string path);

public:
    //Motor(string pin,float coef_ang, float coef_lin);
    Motor(string pin,float coef_a, float coef_b, float coef_c);

    void forceinNewtonSaturation(float &force);
    void forceinPorcentSaturation(float &force);

    int forceinNewton2duty(float force);
    int forceinPorcent2duty(float force);

    void setInitialMovement();
    void setForceinNewton(float force);
    void setRelativeForceinNewton(float force);
    void setForceinPorcent(float force);

    void setTrimForceinNewtonas(float forceTRIM);

    void stop();
    void deactivate();


    ~Motor();

};


#endif /* MOTOR_H */

