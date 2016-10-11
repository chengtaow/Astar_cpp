/* 
 * Chengtao Wang, 10/10/2016
 * ctwang28@gmail.com
*/
#include "Astar.hpp"
#include <vector>

using namespace std;

int main(){
	//Define the size of map, the start, the goal, and the obstacles.
	Astar apath(9, 0, 80);
	int ob[] = {2, 11, 20, 29, 38, 47, 23, 32, 41, 50, 59, 68, 77};
	vector<int> obstacle(&ob[0], &ob[13]);
	//Astar
	apath.generate_obstacle(obstacle);
	apath.astar_algorithm();
	apath.print_map();
	
	return 0;
}