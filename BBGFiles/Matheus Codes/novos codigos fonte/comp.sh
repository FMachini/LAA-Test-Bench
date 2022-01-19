#!/bin/bash

g++ -c lqrmat.cpp
g++ -c server.cpp

g++ -o server lqrmat.o server.o -O2 -larmadillo

#g++ server.cpp -o server
#g++ client.cpp -o client
