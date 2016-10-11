/* 
 * Chengtao Wang, 10/10/2016
 * ctwang28@gmail.com
*/
#include <map>
#include <vector>
#include <iostream>
#include <iomanip>
#include <cmath>
#include <algorithm>
#include "Astar.hpp"

using namespace std;

Astar::Astar(int ms, int st, int go){
	//The constructor, generate the grid map, initialize some values.
	this->map_size = ms;
	this->start = st;
	this->goal = go;
	int nodenumber = 0;
	for (int i = 0; i < map_size; i++){
		for (int j = 0; j < map_size; j++){
			this->grid_map.insert(pair<int, char> (nodenumber, '.'));
			nodenumber++;
		}
	}
	// '.' for not filled, '#' for filled, '@' for path, '+' for start and goal
	
	this->Openset.push_back(this->start);
	for(int i = 0; i < nodenumber; i++){
		this->gscore.insert(pair<int, double> (i, 1000));
		this->fscore.insert(pair<int, double> (i, 1000));
	}
	this->gscore[this->start] = 0;
	this->fscore[this->start] = this->heuristic_cost(this->start, this->goal);

}

int* Astar::getcoor(int node){
	//Get the cooridinate of a node
	int *coor;
	coor = new int[2];
	coor[0] = node%this->map_size+1;
	coor[1] = node/this->map_size+1;
	return coor;
}

double Astar::heuristic_cost(int from, int to){
	//Compute the heuristic cost between two nodes
	int *coor_f;
	int *coor_t;
	double cost;
	coor_f = this->getcoor(from);
	coor_t = this->getcoor(to);
	cost = sqrt(pow((coor_f[0] - coor_t[0]),2) + pow((coor_f[1] - coor_t[1]),2));
	return cost;
}

bool Astar::isneighbor(int node){
	//Whether the neighbor of the node is available
	if (this->grid_map[node] == '#') {return 0;}
	else {return 1;}
}

vector<int> Astar::Getallneighbor(int current){
	//Generate all neibor nodes of the current
	vector<int> allneighbor;
	int gs = this->map_size;
	int *coor;
	coor = getcoor(current);
	if (coor[0] < gs){
		if (isneighbor(current + 1)) {allneighbor.push_back(current + 1);}
	}
	if (coor[0] > 1){
		if (isneighbor(current - 1)) {allneighbor.push_back(current - 1);}
	}
	if (coor[1] < gs){
		if (isneighbor(current + gs)) {allneighbor.push_back(current + gs);}
	}
	if (coor[1] > 1){
		if (isneighbor(current - gs)) {allneighbor.push_back(current - gs);}
	}
	if (coor[1] > 1 && coor[0] > 1){
		if (isneighbor(current - gs - 1)) {allneighbor.push_back(current - gs - 1);}
	}
	if (coor[1] > 1 && coor[0] < gs){
		if (isneighbor(current - gs + 1)) {allneighbor.push_back(current - gs + 1);}
	}
	if (coor[1] < gs && coor[0] > 1){
		if (isneighbor(current + gs - 1)) {allneighbor.push_back(current + gs - 1);}
	}
	if (coor[1] < gs && coor[0] < gs){
		if (isneighbor(current + gs + 1)) {allneighbor.push_back(current + gs + 1);}
	}

	return allneighbor;
}

void Astar::generate_obstacle(vector<int> obstacle){
	//Set the wall
	char fill = '#';
	int n = obstacle.size();
	for(int i = 0; i < n; i++){
		this->grid_map[obstacle[i]] = fill;
	}
}

void Astar::astar_algorithm(){
	//The main Astar algorithm
	int path = 0;//Wheter there is a path.
	while ( !this->Openset.empty() ){
		//Current = the node in Openset having the least fscore value.
		int current;
		double leastf = 1000;
		for (int i = 0; i < this->Openset.size();i++){
			if (this->fscore[this->Openset[i]] < leastf) {
				leastf = this->fscore[this->Openset[i]];
				current = this->Openset[i];
			}
		}

		if (current == this->goal){
			path = 1;
			this->reconstruct_path(current);
		}
		//cout << "Current is: " << current  << endl;

		//Remove current from Openset
		vector<int>::iterator itr = this->Openset.begin();
		while (itr != this->Openset.end()){
			if (*itr == current){this->Openset.erase(itr);break;}
			itr++;
		}
		//Add current to Closedset
		this->Closedset.push_back(current);
		//cout<<"Add current to Closedset" << current<<endl;

		//For each neighbor of Current, figure out all the neighbor.
		vector<int> neighbor = this->Getallneighbor(current);
		//Call the function Getallneighbor
		for(int i = 0; i < neighbor.size(); i++){
			//cout<< "Neighbor is"<< neighbor[i]<<endl;
			vector<int>::iterator itr2;
			int nodenei = neighbor[i];
			itr2 = find(this->Closedset.begin(), this->Closedset.end(), nodenei);
			if (itr2 != this->Closedset.end()) {continue;} //Ignore already evaluated neighbor

			double t_gscore = this->gscore[current] + this->heuristic_cost(current, nodenei);

			itr2 = find(this->Openset.begin(), this->Openset.end(), nodenei);
			if (itr2 == this->Openset.end()) {this->Openset.push_back(nodenei);}//Discover new node
			else if (t_gscore >= this->gscore[nodenei]) {continue;}//Not a better path

			this->camefrom.insert(pair<int, int> (nodenei, current));
			this->gscore[nodenei] = t_gscore;
			this->fscore[nodenei] = t_gscore + this->heuristic_cost(nodenei, goal);
		}

	}
	
	if (path == 0){
		cout << "There is no path to the goal!" << endl;
	}
}

void Astar::reconstruct_path(int current){
	//Generate the Astar path
	vector<int> total_path;
	total_path.push_back(current);
	while (1){
		current = this->camefrom[current];
		total_path.push_back(current);
		if (current == this->start){break;}
	}
	cout<< "Get path"<<endl;

	for (int i = 0; i < total_path.size(); i++){
		char path = '@';
		this->grid_map[total_path[i]] = path;
	}
}

void Astar::print_map(){
	//Print the map
	//set the start and goal
	char startgoal = '+';
	this->grid_map[this->start] = startgoal;
	this->grid_map[this->goal] = startgoal;
	
	int k=0;
	for (int i = 0; i < this->map_size; i++){
		for (int j = 0; j < this->map_size; j++){
			cout<< this->grid_map[k]<<' ';
			k++;
		}
		cout<<endl;
	}
	cout<<endl;

	//Print number of the grids
	k=0;
	for (int i = 0; i < this->map_size; i++){
		for (int j = 0; j < this->map_size; j++){
			cout<< setw(2) << k <<' ';
			k++;
		}
		cout<<endl;
	}
	cout<<endl;

}

