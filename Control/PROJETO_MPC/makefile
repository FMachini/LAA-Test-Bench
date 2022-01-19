-include subdir.mk

CXX := g++
CXX_FLAGS := -Wall -Wextra -std=c++17 -ggdb

BIN := bin
SRC := src
INCLUDE := 

LIBRARIES :=-L. -lMPC -larmadillo -llapack -lblas
EXECUTABLE := executavel


all:	$(EXECUTABLE)

run:	
	clear	
	@echo "Executing..."
	./$(EXECUTABLE)

$(EXECUTABLE): 	$(OBJS)
		@echo "Building..."
		$(CXX) -o $@ $< $(LIBRARIES)
		@echo 'Finished building target: $@'
clean:
	@echo "Clearing..."
	rm *.o