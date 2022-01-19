#ifndef Servo_H
#define Servo_H

#include <iostream>
#include <fstream>
#include <vector>
#include <stdio.h>
#include <string>
#include <math.h>
using namespace std;
#define rad2deg (180/3.1415)
#define deg2rad (3.1415/180)

class Servo{

private:
	//float angle{0.0};
	//float duty_cycle{2200000};
	//float period{2200000};
	float angleMax = 35.0;
	float angleMin = 0.0;

	float a;
	float b;
	float c;


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
    void setDutyCycle(int duty);

public:
    Servo(string pin,float coef_a,float coef_b,float coef_c);
    void setValue(int val, string path);
    void setExport(int num_pwm);
    void setPeriod(int period);
    void setEnable(int enable);

//    int angleinRad2duty(float angle);
    int angleinDeg2duty(float angle);

//    void saturationAngleinRad(float &angle);
    void saturationAngleinDeg(float &angle);

//    void setAngleinRad(float angle);
    void setAngleinDeg(float angle);

    void setAngleRange(float angleMAX,float angleMIN);

    void deactivate();
    ~Servo();
};


#endif /* Servo_H */

