################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
simAtitudeMPC.cpp 

OBJS += \
simAtitudeMPC.o 

CPP_DEPS += \
simAtitudeMPC.d 


# Each subdirectory must supply rules for building sources it contributes
%.o: %.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	@g++ -std=c++11 -I ../include\
	    -I ./qpOASES/include\
	    -include ./qpOASES/src/Bounds.cpp\
	    -include ./qpOASES/src/QProblem.cpp\
	    -include ./qpOASES/src/BLASReplacement.cpp\
	    -include ./qpOASES/src/Constraints.cpp\
	    -include ./qpOASES/src/Flipper.cpp\
	    -include ./qpOASES/src/Indexlist.cpp\
	    -include ./qpOASES/src/LAPACKReplacement.cpp\
	    -include ./qpOASES/src/Matrices.cpp\
	    -include ./qpOASES/src/MessageHandling.cpp\
	    -include ./qpOASES/src/Options.cpp\
	    -include ./qpOASES/src/OQPinterface.cpp\
	    -include ./qpOASES/src/QProblemB.cpp\
	    -include ./qpOASES/src/SolutionAnalysis.cpp\
	    -include ./qpOASES/src/SparseSolver.cpp\
	    -include ./qpOASES/src/SQProblem.cpp\
	    -include ./qpOASES/src/SubjectTo.cpp\
	    -include ./qpOASES/src/SQProblemSchur.cpp\
	    -include ./qpOASES/src/Utils.cpp\
	    -O2 -fwhole-program -c\
	    -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ''


