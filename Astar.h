//
//  Astar.h
//  Astar
//
//  Created by Tianhao Ye on 10/15/16.
//  Copyright © 2016 Tianhao Ye. All rights reserved.
//

#ifndef Astar_h
#define Astar_h

#include <vector>	// std::vector
#include <cfloat>	// DBL_MAX
#include <algorithm>	//std::find
#include <cmath> //
#include "GridMap.h"

// using namespace std;

struct Node
{
	size_t idx, x, y, cameFrom;
	double gScore, fScore;
	Node() {}
	Node(size_t i) : idx(i), gScore(DBL_MAX), fScore(DBL_MAX) {}
};

class Astar{
public:
	Astar(size_t r, size_t c, size_t start, size_t goal);
	Astar(size_t s, size_t start, size_t goal);
	~Astar();
	void setObstacle(std::vector<size_t> obstacle);
	void pathPlanning();
	void showMap();
	void showPath();

private:
	size_t _row, _col, _start, _goal, _nodeNum;
	bool _pathExists;
	GridMap* _map;
	std::vector<size_t> _closedSet, _openSet, _path;
	Node** _nodes;
	void initialize();

	double heuristicCost(size_t from, size_t to);
	bool isObstacle(size_t nodeIdx);
	bool isObstacle(size_t x, size_t y);
	std::vector<size_t> getNeighbor(size_t nodeIdx);

};

#endif /* Astar_h */
