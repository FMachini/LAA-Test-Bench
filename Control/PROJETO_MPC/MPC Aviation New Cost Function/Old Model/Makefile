all: comp clean  run

comp:	main.o I2Cdev.o Kalman.o MPU6050.o getStates.o eqep.o motor.o
	g++ -o teste I2Cdev.o Kalman.o eqep.o MPU6050.o main.o getStates.o motor.o ../../src/BLASReplacement.cpp ../../src/LAPACKReplacement.cpp ../../src/Bounds.cpp ../../src/Constraints.cpp ../../src/Flipper.cpp ../../src/Indexlist.cpp ../../src/Matrices.cpp ../../src/MessageHandling.cpp ../../src/Options.cpp ../../src/QProblemB.cpp ../../src/QProblem.cpp ../../src/SolutionAnalysis.cpp ../../src/SparseSolver.cpp ../../src/SQProblem.cpp ../../src/SQProblemSchur.cpp ../../src/SubjectTo.cpp ../../src/Utils.cpp -I ../../include/  -L ../../bin/ -lqpOASES -std=gnu++11 -larmadillo

main.o: main.cpp
	g++ -c main.cpp ../../src/BLASReplacement.cpp ../../src/LAPACKReplacement.cpp ../../src/Bounds.cpp ../../src/Constraints.cpp ../../src/Flipper.cpp ../../src/Indexlist.cpp ../../src/Matrices.cpp ../../src/MessageHandling.cpp ../../src/Options.cpp ../../src/QProblemB.cpp ../../src/QProblem.cpp ../../src/SolutionAnalysis.cpp ../../src/SparseSolver.cpp ../../src/SQProblem.cpp ../../src/SQProblemSchur.cpp ../../src/SubjectTo.cpp ../../src/Utils.cpp -I ../../include/  -L ../../bin/ -lqpOASES -std=gnu++11

I2Cdev.o: i2c/I2Cdev.cpp i2c/I2Cdev.h
	g++ -c i2c/I2Cdev.cpp -std=gnu++11

Kalman.o: Kalman/Kalman.cpp Kalman/Kalman.h
	g++ -c Kalman/Kalman.cpp -std=gnu++11

MPU6050.o: MPU6050/MPU6050.cpp MPU6050/MPU6050.h
	g++ -c MPU6050/MPU6050.cpp -std=gnu++11
	
eqep.o: eqep.cpp eqep.h
	g++ -c eqep.cpp -std=gnu++11
	
getStates.o: getStates.cpp getStates.h
	g++ -c getStates.cpp -std=gnu++11
	
motor.o: motor.cpp motor.h
	g++ -c motor.cpp -std=gnu++11

clean:
	rm *.o
	
run:
	./getData
