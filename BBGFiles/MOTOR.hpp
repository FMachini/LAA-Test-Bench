#ifndef MOTOR_H
#define MOTOR_H

#include <iostream>
#include <fstream>
#include <vector>
#include <stdio.h>
#include <string>

using namespace std;
#define rad2deg (180/3.1415)
#define deg2rad (3.1415/180)

class Motor{

private:
	float forcePorcentMax = 45;      //In % of force total
	float forcePorcentMin = 30;      //In % of force total

	float forceNewtonMax = 3.20;     //In N
	float forceNewtonMin = 1.39;     //In N

	float trimForceinPorcent = 37;     //In % of force total
	float trimForceinNewton  = 1.93;   //In N

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
    Motor(string pin);

    void forceinNewtonSaturation(float &force);
    void forceinPorcentSaturation(float &force);

    int forceinNewton2duty(float force);
    int forceinPorcent2duty(float force);

    void setForceinNewton(float force);
    void setRelativeForceinNewton(float force);
    void setForceinPorcent(float force);

    void setTrimForceinNewtonas(float forceTRIM);

    void stop();
    void deactivate();


    ~Motor();

};


#endif /* MOTOR_H */

