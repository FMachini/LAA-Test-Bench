################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/Sensor/i2c/I2Cdev.cpp 

OBJS += \
./src/Sensor/i2c/I2Cdev.o 

CPP_DEPS += \
./src/Sensor/i2c/I2Cdev.d 


# Each subdirectory must supply rules for building sources it contributes
src/Sensor/i2c/%.o: ../src/Sensor/i2c/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	arm-linux-gnueabihf-g++ -DARMA_DONT_USE_WRAPPER -I"C:\Users\-F.Machini\Desktop\bancada\Codigos\Codigos\theta4-MPC-restricao\src\qpOASES\include" -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


