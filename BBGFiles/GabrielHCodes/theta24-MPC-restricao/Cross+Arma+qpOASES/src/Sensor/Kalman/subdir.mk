################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/Sensor/Kalman/Kalman.cpp 

OBJS += \
./src/Sensor/Kalman/Kalman.o 

CPP_DEPS += \
./src/Sensor/Kalman/Kalman.d 


# Each subdirectory must supply rules for building sources it contributes
src/Sensor/Kalman/%.o: ../src/Sensor/Kalman/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	arm-linux-gnueabihf-g++ -DARMA_DONT_USE_WRAPPER -I"/home/laa/eclipse-workspace/theta24-MPC-restricao/src/qpOASES/include" -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


