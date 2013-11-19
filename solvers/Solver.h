#pragma once

#include <vector>
#include "skeleton.h"
using namespace std;

class Solver
{
public:
	Solver(void);
	~Solver(void);

	virtual vector<pair<int,Eigen::Vector3d > > solve(double time) = 0;
};

