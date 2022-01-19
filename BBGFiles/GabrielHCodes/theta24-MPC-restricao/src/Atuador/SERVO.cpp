#include "SERVO.hpp"


Servo::Servo(string pin,float coef_a,float coef_b,float coef_c){
	a = coef_a;
	b = coef_b;
	c = coef_c;

	int num_pwm;
	if(pin=="P8.13"){ //servo do tilt 1
		pwm_number = "pwm1/";
		pwmchip_number = "pwmchip6/";
		num_pwm=1;
	}
	else if(pin=="P8.19"){//servo do tilt 2
		pwm_number = "pwm0/";
		pwmchip_number = "pwmchip6/";
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
	setPeriod(20000000); 		// period in nanosegundos
    setAngleinDeg(0.0); 	    // Angle in deg
    setEnable(1); 				// Enable in 1
}

void Servo::setValue(int val1, string path){
   
    ofstream fs;
    fs.open(path.c_str());
    
    if (!fs)
    {
        cerr<< "Servo path is not open"<<endl;
    }
    fs << val1;
    fs.close();

}
void Servo::setExport(int num_pwm){
   setValue(num_pwm,export_path);
}
void Servo::setPeriod(int period){
   setValue(period,period_path);
}

void Servo::setEnable(int enable){
   setValue(enable,enable_path);
}


//void Servo::saturationAngleinRad(float &angle){
//
//	 if (angle > angleMax*deg2rad)
//	 {
//	    angle = angleMax*deg2rad;
//	 }
//	 else if (angle < angleMin*deg2rad)
//	 {
//	    angle= angleMin*deg2rad;
//	 }
//}

void Servo::saturationAngleinDeg(float &angle){

	 if (angle > angleMax)
	 {
	    angle = angleMax;
	 }
	 else if (angle < angleMin)
	 {
	    angle= angleMin;
	 }
}


//int Servo::angleinRad2duty(float angleVec){
//
//    int duty;
//
//    duty = (-18.27632*(angleVec)*rad2deg +  2.2799e+03)*1000; // Tilt 1
//
//    return duty;
//}

int Servo::angleinDeg2duty(float angle){

    int duty;

    duty = ( (-b - sqrt(b*b-4*a*(c - angle)))/(2*a));

    return duty;
}

//void Servo::setAngleinRad(float angle){
//
//	saturationAngleinRad(angle);
//	int duty = angleinRad2duty(angle);
//	setDutyCycle(duty);
//
//}

void Servo::setAngleinDeg(float angle){

	saturationAngleinDeg(angle);
	int duty = angleinDeg2duty(angle);
	setDutyCycle(duty);

}

void Servo::setAngleRange(float angleMAX,float angleMIN){
	angleMax = angleMAX;
	angleMin = angleMIN;
}

void Servo::setDutyCycle(int duty){

    setValue(duty,duty_cycle_path);
}


void Servo::deactivate()
{
	setAngleinDeg(0.0);

	setEnable(0);
}

Servo::~Servo()
{

}


