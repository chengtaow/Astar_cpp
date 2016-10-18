//
//  GridMap.cpp
//  Astar
//
//  Created by Tianhao Ye on 10/15/16.
//  Copyright © 2016 Tianhao Ye. All rights reserved.
//

#include "GridMap.h"

GridMap::GridMap(size_t r, size_t c) : _row(r), _col(c)
{
  initialize();
}

GridMap::GridMap(size_t s) : _row(s), _col(s)
{
  initialize();
}

GridMap::~GridMap()
{
  for (size_t i = 0; i < _row; i++) {
    delete [] grids[i];
  }
  delete [] grids;
  grids = nullptr;
}

void GridMap::initialize()
{
  grids = new char* [_row];
  for (size_t i = 0; i < _row; i++) {
    grids[i] = new char[_col];
    for (size_t j = 0; j < _col; j++) {
      grids[i][j] = '.';
    }
  }
  return;
}

void GridMap::setSymbolAt(size_t x, size_t y, const char symbol)
{
  grids[x][y] = symbol;
  return;
}

void GridMap::setSymbolAt(size_t idx, const char symbol)
{
  size_t x = idx / _col;
  size_t y = idx % _col;
  setSymbolAt(x, y, symbol);
  return;
}

const char GridMap::getSymbolAt(size_t x, size_t y) const
{
  return grids[x][y];
}

const char GridMap::getSymbolAt(size_t idx) const
{
  size_t x = idx / _col;
  size_t y = idx % _col;
  return getSymbolAt(x, y);

}

void GridMap::printMap()
{
	for (size_t i = 0; i < _row; i++){
		for (size_t j = 0; j < _col; j++){
			std::cout << grids[i][j] << ' ' ;
		}
		std::cout << std::endl;
	}
	std::cout << std::endl;

	//Print number of the grids
  for (size_t i = 0; i < _row; i++){
		for (size_t j = 0; j < _col; j++){
			std::cout << i * _col + j << "\t" ;
		}
		std::cout << std::endl;
	}
	std::cout << std::endl;
}
