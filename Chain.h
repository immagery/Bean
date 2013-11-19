#ifndef CHAIN_CPP
#define CHAIN_CPP

#include <vector>
#include <map>
using namespace std;

class Chain {
public:

	Chain() { 
		positions = vector<Eigen::Vector3d>(0); 
		sceneIDToChainID = map<int,int>();
	}


	vector<Eigen::Vector3d> positions;
	map<int,int> sceneIDToChainID;
};

#endif