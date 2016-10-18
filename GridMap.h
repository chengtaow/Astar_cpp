//
//  GridMap.h
//  Astar
//
//  Created by Tianhao Ye on 10/15/16.
//  Copyright © 2016 Tianhao Ye. All rights reserved.
//

#ifndef GridMap_h
#define GridMap_h

#include <iostream>

class GridMap
{
public:
  GridMap(size_t r = 1, size_t c = 1);
  GridMap(size_t s = 1);
  virtual ~GridMap();

  void setSymbolAt(size_t x, size_t y, const char symbol);
  void setSymbolAt(size_t idx, const char symbol);
  const char getSymbolAt(size_t x, size_t y) const;
  const char getSymbolAt(size_t idx) const;

  size_t getRow() const { return _row; }
  size_t getCol() const { return _col; }

  void printMap();

protected:
  size_t _row, _col;
  char** grids;
  void initialize();
};


#endif /* GridMap_h */
