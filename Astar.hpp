/*
 * Chengtao Wang, 10/10/2016
 * ctwang28@gmail.com
*/
#include <map>
#include <vector>

using namespace std;

class Astar{
private:
	int map_size, start, goal;
	map<int, char> grid_map;
	map<int, int> camefrom;
	map<int, double> gscore;
	map<int, double> fscore;
	vector<int> Closedset;
	vector<int> Openset;
	int* getcoor(int node);
	double heuristic_cost(int from, int to);
	bool isneighbor(int node);
	
public:
	Astar(int ms, int st, int go);
	vector<int> Getallneighbor(int current);
	void generate_obstacle(vector<int> obstacle);
	void astar_algorithm();
	void reconstruct_path(int current);
	void print_map();
	
};
