all: clean comp run

comp:	teste.o I2Cdev.o Kalman.o MPU6050.o 
	g++ -o teste3 I2Cdev.o Kalman.o MPU6050.o Teste3.o -std=gnu++11

teste.o: teste.cpp
	g++ -c Teste3.cpp -std=gnu++11

I2Cdev.o: i2c/I2Cdev.cpp i2c/I2Cdev.h
	g++ -c  i2c/I2Cdev.cpp

Kalman.o: Kalman/Kalman.cpp Kalman/Kalman.h
	g++ -c Kalman/Kalman.cpp

MPU6050.o: MPU6050/MPU6050.cpp MPU6050/MPU6050.h
	g++ -c MPU6050/MPU6050.cpp

clean:
	rm *.o

run: 
	./teste
