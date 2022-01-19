all: main

main:	main.o lqrmat.o Comm.o Motor.o
	g++ -o main lqrmat.o Comm.o Motor.o main.o -larmadillo

main.o: main.cpp
	g++ -c main.cpp

Motor.o: motor/Motor.cpp motor/Motor.hpp
	g++ -c motor/Motor.cpp

Comm.o: comm/Comm.cpp comm/Comm.hpp
	g++ -c comm/Comm.cpp

lqrmat.o: control/lqrmat.cpp control/lqrmat.hpp
	g++ -c control/lqrmat.cpp -O2 -larmadillo

clean:
	rm *.o
