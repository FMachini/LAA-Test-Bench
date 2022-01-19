#include <iostream>
#include <stdlib.h> //exit(...)
#include <cstdio>		//sprintf

//file I/O
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdio.h>
#include <sys/types.h>
#include <unistd.h>
#include <stdio.h>

//signal handler - ctrl + C - parada de emergencia.
#include <signal.h>
#include <stdlib.h>


#define path_export "/sys/class/pwm/pwmchip0/pwm0/export"
#define path_enable "/sys/class/pwm/pwmchip0/pwm0/enable"
#define path_period "/sys/class/pwm/pwmchip0/pwm0/period"
#define path_duty_cycle "/sys/class/pwm/pwmchip0/pwm0/duty_cycle"

using namespace std;

void setValue(int value, const char* path);
//void setPWM(const char* path);
void activateEsc();
	
int main()
{
	int PWM;
	int value;
	const char* path;
	
	activateEsc();
		
	while(1){
	
	cout << "Input the desired Duty-cycle: " << endl;
	scanf("%d", &PWM);
	cout << "Duty-cycle set to " << PWM << endl;
	setValue(PWM, path_duty_cycle);	
	
	}
	
	return 0;
}

void activateEsc(){
	
	int string_export = 1;
	int string_enable = 1;
	int string_period = 3000000;
	int string_duty_cycle = 1000000;
	
	// activating PWM port
	
	setValue(string_export, path_export);
	setValue(string_enable, path_enable);
	setValue(string_period, path_period);
	setValue(string_duty_cycle, path_duty_cycle);
		
	
	cout << "Esc Engaged! Duty-Cycle set to " << string_duty_cycle << endl;
}

void setValue(int value, const char* path){
	
	char string[20];
	sprintf(string, "%d", value);
	int file;
	file = open(path, O_RDWR | O_CREAT);
	write(file, string, sizeof(string));
	close(file);	
	
}

//void setPWM(const char* path, int PWM){
//	write_value(PWM, path);	
//}
