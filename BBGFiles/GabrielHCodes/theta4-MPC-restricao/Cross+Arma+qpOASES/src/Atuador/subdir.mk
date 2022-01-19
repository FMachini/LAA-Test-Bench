################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/Atuador/MOTOR.cpp \
../src/Atuador/SERVO.cpp 

OBJS += \
./src/Atuador/MOTOR.o \
./src/Atuador/SERVO.o 

CPP_DEPS += \
./src/Atuador/MOTOR.d \
./src/Atuador/SERVO.d 


# Each subdirectory must supply rules for building sources it contributes
src/Atuador/%.o: ../src/Atuador/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	arm-linux-gnueabihf-g++ -DARMA_DONT_USE_WRAPPER -I"C:\Users\-F.Machini\Desktop\bancada\Codigos\Codigos\theta4-MPC-restricao\src\qpOASES\include" -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


