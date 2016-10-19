CXX=g++

all: app

app: main.cpp Astar.h GridMap.h Astar.cpp GridMap.cpp
	$(CXX) -Wall -std=c++11 main.cpp Astar.cpp GridMap.cpp -o app

.PHONY: clean
clean:
	rm app
