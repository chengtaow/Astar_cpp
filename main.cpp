//
//  main.cpp
//  Astar
//
//  Created by Tianhao Ye on 10/15/16.
//  Copyright © 2016 Tianhao Ye. All rights reserved.
//

#include "Astar.h"
#include <vector>

using namespace std;

int main(){
	//Define the size of map, the start, the goal, and the obstacles.
	Astar apath(9, 0, 80);
	vector<size_t> obstacle= {2, 11, 20, 29, 38, 47, 23, 32, 41, 50, 59, 68, 77};
	//Astar
	apath.setObstacle(obstacle);
	apath.showMap();
	apath.pathPlanning();
	apath.showMap();
	apath.showPath();

	return 0;
}
