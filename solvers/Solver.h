#pragma once

#include <vector>
#include "skeleton.h"
using namespace std;

class Solver
{
public:
	Solver(void);
	~Solver(void);

	virtual vector<pair<int,Eigen::Quaternion<double> > > solve(double time) = 0;

};

