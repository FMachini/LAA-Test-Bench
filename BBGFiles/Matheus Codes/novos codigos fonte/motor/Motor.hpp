#ifndef MOTOR_HPP
#define MOTOR_HPP

#include <>

class Motor
{
	public:
		Motor() :
			pathM1_(),		
			pathM1_(),
			pathM1_(),
			pathM1_();	

		void setPWM(int pwm); //checar se pwm é positivo senao encerrar - erro!
		

	private:
		const char* pathM1_ = "/sys/class/pwm/pwmchip0/pwm0/duty_cycle";
		const char* pathM2_ = "/sys/class/pwm/pwmchip3/pwm0/duty_cycle";
		const char* pathM3_ = "/sys/class/pwm/pwmchip1/pwm0/duty_cycle";
		const char* pathM4_ = "/sys/class/pwm/pwmchip1/pwm1/duty_cycle";
};

#endif
