all: clean comp run

comp:	main.o lqrmat.o Comm.o Motor.o Emergency.o
	g++ -o main lqrmat.o Comm.o Motor.o Emergency.o main.o -larmadillo

main.o: main.cpp
	g++ -c main.cpp

Emergency.o: emergency/Emergency.cpp emergency/Emergency.hpp
	g++ -c  emergency/Emergency.cpp

Motor.o: motor/Motor.cpp motor/Motor.hpp
	g++ -c motor/Motor.cpp

Comm.o: comm/Comm.cpp comm/Comm.hpp
	g++ -c comm/Comm.cpp

lqrmat.o: control/lqrmat.cpp control/lqrmat.hpp
	g++ -c control/lqrmat.cpp -O2 -larmadillo

clean:
	rm *.o

run: 
	./main
