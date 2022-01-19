all: clean comp run

comp:	main.o encoder.o lqrmat.o Comm.o Motor.o Emergency.o
	g++ -o main eqep.o lqrmat.o Comm.o Motor.o Emergency.o main.o -larmadillo -std=gnu++11

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

clean:
	rm *.o

run: 
	./main
