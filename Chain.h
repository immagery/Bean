#ifndef CHAIN_CPP
#define CHAIN_CPP

#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace std;
using namespace Eigen;

class Chain {
public:

	Chain() { 
		positions = vector<Eigen::Vector3d>(0); 
	}


	vector<Vector3d> positions;
};

#endif