#ifndef CHAIN_CPP
#define CHAIN_CPP

#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace std;
using namespace Eigen;

class Chain {
public:

	Chain() 
	{ 
		positions.clear();
		normalVectors.clear();
	}

	Chain(int initSize) 
	{ 
		positions.resize(initSize);
		normalVectors.clear();
	}

	vector<Vector3d> positions;
	vector<Vector3d> normalVectors;
};

#endif