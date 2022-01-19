all: server

server:	server.o lqrmat.o
	g++ -o server lqrmat.o  server.o -larmadillo

server.o: server.cpp
	g++	-c server.cpp

lqrmat.o: lqrmat.cpp lqrmat.hpp
	g++	-c lqrmat.cpp -O2 -larmadillo

clean:
	rm *.o
