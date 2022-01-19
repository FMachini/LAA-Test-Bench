all: comp clean  run

comp:	main.o I2Cdev.o Kalman.o MPU6050.o getStates.o eqep.o motor.o
	g++ -o getData  I2Cdev.o Kalman.o eqep.o MPU6050.o main.o getStates.o motor.o -std=gnu++11

main.o: main.cpp
	g++ -c main.cpp -std=gnu++11

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
