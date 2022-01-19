all: clean comp run

comp:	main.o MPU.o I2C.o encoder.o lqrmat.o Comm.o Motor.o Emergency.o
	g++ -o main MPU6050.o I2Cdev.o eqep.o lqrmat.o Comm.o Motor.o Emergency.o main.o -larmadillo -std=gnu++11

main.o: main.cpp
	g++ -c main.cpp -std=gnu++11

Emergency.o: emergency/Emergency.cpp emergency/Emergency.hpp
	g++ -c emergency/Emergency.cpp -std=gnu++11

Motor.o: motor/Motor.cpp motor/Motor.hpp
	g++ -c motor/Motor.cpp -std=gnu++11

Comm.o: comm/Comm.cpp comm/Comm.hpp
	g++ -c comm/Comm.cpp -std=gnu++11

lqrmat.o: control/lqrmat.cpp control/lqrmat.hpp
	g++ -c control/lqrmat.cpp -O2 -larmadillo -std=gnu++11

encoder.o: encoder/eqep.cpp encoder/eqep.h
	g++ -c encoder/eqep.cpp -std=gnu++11

I2C.o: imu/I2C/I2Cdev.cpp imu/I2C/I2Cdev.h
	g++ -c  imu/I2C/I2Cdev.cpp

MPU.o: imu/MPU6050/MPU6050.cpp imu/MPU6050/MPU6050.h 
	g++ -c imu/MPU6050/MPU6050.cpp

clean:
	rm *.o

run: 
	./main
