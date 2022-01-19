#include "MOTOR.hpp"

Motor::Motor(string pin){
	int num_pwm;
	if(pin=="P9.42"){
		pwm_number = "pwm-7:1/";
		pwmchip_number = "pwmchip7/";
		num_pwm=1;
	}
	else if(pin=="P9.22"){
		pwm_number = "pwm-7:0/";
		pwmchip_number = "pwmchip7/";
		num_pwm=0;
	}
	else{
		cerr << "Pin not okay"<< endl;
	}

	export_path = root +pwmchip_number + exp;
	period_path = root + pwmchip_number + pwm_number + period;
	duty_cycle_path =root + pwmchip_number + pwm_number + duty_cycle;
	enable_path =root + pwmchip_number + pwm_number + enable;

	setExport(num_pwm);
	setPeriod(3333333);     				// period in nanoseconds we want 300 Hz
	setDutyCycle(1000000);					// Initialize the esc
	setEnable(1); 		    				// Enable in 1
}

void Motor::setExport(int num_pwm){
   setValue(num_pwm,export_path);
}
void Motor::setPeriod(int period){
   setValue(period,period_path);
}

void Motor::setEnable(int enable){
   setValue(enable,enable_path);
}

void Motor::setValue(int value, string path){
   
    ofstream fs;
    fs.open(path.c_str());
    
    if (!fs)
    {
        std::cerr<< "Motor path is not open"<<std::endl;
    }
    fs << value;
    fs.close();

}

int Motor::forceinNewton2duty(float force){
	int duty = 1000000 + ( (force + 1.39 )/0.0917)*10000;
	return duty;
}

int Motor::forceinPorcent2duty(float force){
	//Encontrar relação e colocar aqui
	int duty = 0;
	return duty;
}

void Motor::forceinPorcentSaturation(float &force){
	 //Remember we are using force in % of the total force
	 if (force < forcePorcentMin)
	 {
	    force = forcePorcentMin;
	 } else if (force > forcePorcentMax)
	 {
	    force = forcePorcentMax;
	 }
}

void Motor::forceinNewtonSaturation(float &force){
	 if (force < forceNewtonMin)
	 {
	    force = forceNewtonMin;
	 } else if (force > forceNewtonMax)
	 {
	    force = forceNewtonMax;
	 }
}

void Motor::setDutyCycle(int duty){
    setValue(duty,duty_cycle_path);
}

void Motor::setForceinNewton(float force){
	forceinNewtonSaturation(force);
    int duty = forceinNewton2duty(force);
	setDutyCycle(duty);
}

void Motor::setRelativeForceinNewton(float force){
	float forceAbs = force + trimForceinNewton;
	forceinNewtonSaturation(forceAbs);
    int duty = forceinNewton2duty(forceAbs);
	setDutyCycle(duty);
}

void Motor::setForceinPorcent(float force){
	forceinPorcentSaturation(force);
    int duty = forceinPorcent2duty(force);
	setDutyCycle(duty);
}

void Motor::setTrimForceinNewtonas(float force){

	trimForceinNewton = force;
}

void Motor::deactivate()
{
	stop();
	setEnable(0);
 }
void Motor::stop(){
	setDutyCycle(1);
}

Motor::~Motor(){

}
