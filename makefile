CXX = g++
CXX_FLAGS = -Wall -std=c++11 -o

all: 8Puzzle FindPath MinPathToBucharest

8Puzzle:
	$(CXX) $(CXX_FLAGS) 8Puzzle.o 8-puzzle/8Puzzle.cpp

FindPath:
	$(CXX) $(CXX_FLAGS) FindPath.o find-path/FindPath.cpp

MinPathToBucharest:
	$(CXX) $(CXX_FLAGS) MinPathToBucharest.o min-path-to-Bucharest/MinPathToBucharest.cpp

clean:
	rm *.o