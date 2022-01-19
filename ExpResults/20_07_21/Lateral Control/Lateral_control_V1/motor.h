#ifndef MOTOR_H
#define MOTOR_H

#include <iostream>
#include <fstream>
#include <vector>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdio.h>
#include <sys/types.h>
#include <unistd.h>
#include <stdio.h>
#include <string>
using namespace std;


class Motor{

private:
    vector<string> paths; 
    char str_val[20];
    string path;
    string exp {"export"}; string enable {"enable"}; 
    string period {"period"}; 
    string duty {"duty_cycle"};
    string root {"/sys/class/pwm/"};

public:
    Motor(string pwmchip, int exp);    
    void setValue(int val, string path);
    void setDutyCycle(int duty);
    //char str2char(string str);
    void deactivateESC();
};


#endif /* MOTOR_H */

