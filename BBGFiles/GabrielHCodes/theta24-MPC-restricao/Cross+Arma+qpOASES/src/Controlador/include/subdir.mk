################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/Controlador/include/matrizesMPC.cpp 

OBJS += \
./src/Controlador/include/matrizesMPC.o 

CPP_DEPS += \
./src/Controlador/include/matrizesMPC.d 


# Each subdirectory must supply rules for building sources it contributes
src/Controlador/include/%.o: ../src/Controlador/include/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	arm-linux-gnueabihf-g++ -DARMA_DONT_USE_WRAPPER -I"/home/laa/eclipse-workspace/theta24-MPC-restricao/src/qpOASES/include" -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


