//
//  Astar.cpp
//  Astar
//
//  Created by Tianhao Ye on 10/15/16.
//  Copyright © 2016 Tianhao Ye. All rights reserved.
//

#include "Astar.h"

Astar::Astar(size_t r, size_t c, size_t start, size_t goal) :
	_row(r), _col(c), _start(start), _goal(goal)
{
	initialize();
}

Astar::Astar(size_t s, size_t start, size_t goal) :
	_row(s), _col(s), _start(start), _goal(goal)
{
	initialize();
}

Astar::~Astar()
{
	delete _map;
	_map = nullptr;
	for (size_t i = 0; i < _nodeNum; i++) {
		delete _nodes[i];
	}
	delete [] _nodes;
	_nodes = nullptr;
}

void Astar::initialize()
{
	// generate a new map
	_map = new GridMap(_row, _col);
	// generate all the nodes
	_nodeNum = _row * _col;
	_nodes = new Node* [_nodeNum];
	for (size_t i = 0; i < _nodeNum; i++) {
		_nodes[i] = new Node(i);
		_nodes[i]->x = i / _col;
		_nodes[i]->y = i % _col;
	}
	// set the start
	_nodes[_start]->gScore = 0;
	_nodes[_start]->fScore = heuristicCost(_start, _goal);
	_path.push_back(_start);
	_openSet.push_back(_start);
	// plot start and goal on the map
	_map->setSymbolAt(_start, '*');
	_map->setSymbolAt(_goal, '&');
	_pathExists = false;
}

double Astar::heuristicCost(size_t from, size_t to){
	//Compute the heuristic cost between two nodes
	double cost;
	cost = sqrt(pow((_nodes[from]->x - _nodes[to]->x),2) + pow((_nodes[from]->y - _nodes[to]->y),2));
	return cost;
}

bool Astar::isObstacle(size_t x, size_t y){
	//Whether the neighbor of the node is available
	return _map->getSymbolAt(x, y) == '#';
}

bool Astar::isObstacle(size_t nodeIdx){
	//Whether the neighbor of the node is available
	return _map->getSymbolAt(nodeIdx) == '#';
}

std::vector<size_t> Astar::getNeighbor(size_t nodeIdx)
{
	std::vector<size_t> neighbors;
	if (_nodes[nodeIdx]->y > 0)  {
		if (!isObstacle(nodeIdx - 1)) {
			neighbors.push_back(nodeIdx - 1);
		}
		if (_nodes[nodeIdx]->x > 0 && !isObstacle(nodeIdx - _col - 1)) {
			neighbors.push_back(nodeIdx - _col - 1);
		}
		if (_nodes[nodeIdx]->x < (_row - 1) && !isObstacle(nodeIdx + _col - 1)) {
			neighbors.push_back(nodeIdx + _col - 1);
		}
	}
	if (_nodes[nodeIdx]->y < (_col - 1)) {
		if (!isObstacle(nodeIdx + 1)) {
			neighbors.push_back(nodeIdx + 1);
		}
		if (_nodes[nodeIdx]->x > 0 && !isObstacle(nodeIdx - _col + 1)) {
			neighbors.push_back(nodeIdx - _col + 1);
		}
		if (_nodes[nodeIdx]->x < (_row - 1) && !isObstacle(nodeIdx + _col + 1)) {
			neighbors.push_back(nodeIdx + _col + 1);
		}
	}
	if (_nodes[nodeIdx]->x > 0 && !isObstacle(nodeIdx - _col)) {
		neighbors.push_back(nodeIdx - _col);
	}

	if (_nodes[nodeIdx]->x < (_row - 1) && !isObstacle(nodeIdx + _col)) {
		neighbors.push_back(nodeIdx + _col);
	}

	return neighbors;
}


void Astar::setObstacle(std::vector<size_t> obstacle){
	// plot the wall
	for(size_t i = 0; i < obstacle.size(); i++){
		_map->setSymbolAt(obstacle[i], '#');
	}
	return;
}

void Astar::pathPlanning()
{
	_pathExists = false;
	while (!_openSet.empty()) {
		double lowestF = DBL_MAX;
		size_t cur;
		for (auto it = _openSet.begin(); it != _openSet.end(); it++) {
			if (_nodes[*it]->fScore < lowestF) {
				lowestF = _nodes[*it]->fScore;
				cur = *it;
			}
		}
		if (cur == _goal) {
			_pathExists = true;
			break;
		} else {
			auto it = find(_openSet.begin(), _openSet.end(), cur);
			_openSet.erase(it);
			_closedSet.push_back(cur);
			std::vector<size_t> neighbors = getNeighbor(cur);
			for (size_t i = 0; i < neighbors.size(); i++) {
				auto it = find(_closedSet.begin(), _closedSet.end(), neighbors[i]);
				if (it == _closedSet.end()) {
					double tentativeGScore = _nodes[cur]->gScore + heuristicCost(cur, neighbors[i]);
					auto it = find(_openSet.begin(), _openSet.end(), neighbors[i]);
					if (it == _openSet.end()) {
						_openSet.push_back(neighbors[i]);
					} else if (tentativeGScore >= _nodes[neighbors[i]]->gScore) {
						continue;
					}
					_nodes[neighbors[i]]->cameFrom = cur;
					_nodes[neighbors[i]]->gScore = tentativeGScore;
					_nodes[neighbors[i]]->fScore = tentativeGScore + heuristicCost(neighbors[i], _goal);

				}
			}
		}
	}
	if (_pathExists) {
		_path.push_back(_goal);
		size_t cur = _nodes[_goal]->cameFrom;
		while (cur != _start) {
			_path.push_back(cur);
			size_t previous = _nodes[cur]->cameFrom;
			if (_nodes[cur]->x == _nodes[previous]->x) {
				_map->setSymbolAt(cur, '-');
			} else if (_nodes[cur]->y == _nodes[previous]->y) {
				_map->setSymbolAt(cur, '|');
			} else if (_nodes[cur]->x < _nodes[previous]->x ) {
				if (_nodes[cur]->y < _nodes[previous]->y) _map->setSymbolAt(cur, '\\');
				else _map->setSymbolAt(cur, '/');
			} else {
				if (_nodes[cur]->y < _nodes[previous]->y) _map->setSymbolAt(cur, '/');
				else _map->setSymbolAt(cur, '\\');
			}
			cur = previous;
		}
		std::cout << "Path planned." << std::endl;
	} else {
		std::cout << "There is no path to the destination!" << std::endl;
	}
	return;
}

void Astar::showMap()
{
	_map->printMap();
	return;
}

void Astar::showPath()
{
	if (_pathExists) {
		std::cout << "Path: " << _start;
		for (auto it = _path.begin() + 1; it != _path.end(); it++) {
			std::cout << "->" << *it;
		}
		std::cout << std::endl;
	} else {
		std::cout << "There is no path to the destination!" << std::endl;

	}
	return;
}
