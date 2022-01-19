################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/qpOASES/src/BLASReplacement.cpp \
../src/qpOASES/src/Bounds.cpp \
../src/qpOASES/src/Constraints.cpp \
../src/qpOASES/src/Flipper.cpp \
../src/qpOASES/src/Indexlist.cpp \
../src/qpOASES/src/LAPACKReplacement.cpp \
../src/qpOASES/src/Matrices.cpp \
../src/qpOASES/src/MessageHandling.cpp \
../src/qpOASES/src/OQPinterface.cpp \
../src/qpOASES/src/Options.cpp \
../src/qpOASES/src/QProblem.cpp \
../src/qpOASES/src/QProblemB.cpp \
../src/qpOASES/src/SQProblem.cpp \
../src/qpOASES/src/SQProblemSchur.cpp \
../src/qpOASES/src/SolutionAnalysis.cpp \
../src/qpOASES/src/SparseSolver.cpp \
../src/qpOASES/src/SubjectTo.cpp \
../src/qpOASES/src/Utils.cpp 

O_SRCS += \
../src/qpOASES/src/BLASReplacement.o \
../src/qpOASES/src/Bounds.o \
../src/qpOASES/src/Constraints.o \
../src/qpOASES/src/Flipper.o \
../src/qpOASES/src/Indexlist.o \
../src/qpOASES/src/LAPACKReplacement.o \
../src/qpOASES/src/Matrices.o \
../src/qpOASES/src/MessageHandling.o \
../src/qpOASES/src/OQPinterface.o \
../src/qpOASES/src/Options.o \
../src/qpOASES/src/QProblem.o \
../src/qpOASES/src/QProblemB.o \
../src/qpOASES/src/SQProblem.o \
../src/qpOASES/src/SQProblemSchur.o \
../src/qpOASES/src/SolutionAnalysis.o \
../src/qpOASES/src/SparseSolver.o \
../src/qpOASES/src/SubjectTo.o \
../src/qpOASES/src/Utils.o 

OBJS += \
./src/qpOASES/src/BLASReplacement.o \
./src/qpOASES/src/Bounds.o \
./src/qpOASES/src/Constraints.o \
./src/qpOASES/src/Flipper.o \
./src/qpOASES/src/Indexlist.o \
./src/qpOASES/src/LAPACKReplacement.o \
./src/qpOASES/src/Matrices.o \
./src/qpOASES/src/MessageHandling.o \
./src/qpOASES/src/OQPinterface.o \
./src/qpOASES/src/Options.o \
./src/qpOASES/src/QProblem.o \
./src/qpOASES/src/QProblemB.o \
./src/qpOASES/src/SQProblem.o \
./src/qpOASES/src/SQProblemSchur.o \
./src/qpOASES/src/SolutionAnalysis.o \
./src/qpOASES/src/SparseSolver.o \
./src/qpOASES/src/SubjectTo.o \
./src/qpOASES/src/Utils.o 

CPP_DEPS += \
./src/qpOASES/src/BLASReplacement.d \
./src/qpOASES/src/Bounds.d \
./src/qpOASES/src/Constraints.d \
./src/qpOASES/src/Flipper.d \
./src/qpOASES/src/Indexlist.d \
./src/qpOASES/src/LAPACKReplacement.d \
./src/qpOASES/src/Matrices.d \
./src/qpOASES/src/MessageHandling.d \
./src/qpOASES/src/OQPinterface.d \
./src/qpOASES/src/Options.d \
./src/qpOASES/src/QProblem.d \
./src/qpOASES/src/QProblemB.d \
./src/qpOASES/src/SQProblem.d \
./src/qpOASES/src/SQProblemSchur.d \
./src/qpOASES/src/SolutionAnalysis.d \
./src/qpOASES/src/SparseSolver.d \
./src/qpOASES/src/SubjectTo.d \
./src/qpOASES/src/Utils.d 


# Each subdirectory must supply rules for building sources it contributes
src/qpOASES/src/%.o: ../src/qpOASES/src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	arm-linux-gnueabihf-g++ -DARMA_DONT_USE_WRAPPER -I"C:\Users\-F.Machini\Desktop\bancada\Codigos\Codigos\theta4-MPC-restricao\src\qpOASES\include" -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


