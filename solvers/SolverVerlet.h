#pragma once
#include "Solver.h"
#include "skeleton.h"

class SolverVerlet : public Solver {
public:
	vector<vector< Vector3d> > restPositions;		// resting positions to compute the desired rest distance between links
	vector<vector< Vector3d> > lastPositions;					// positions in last time
	vector<vector< Vector3d> > currentPositions;
	vector<vector< Vector3d> > idealPositions;

	double lastTime, g, velocityDamping, fps;
	bool hasGravity;

	// Spring parameters: Strength, Damping, Stiffness
	double distS, distD, distStiff;		// springs between nodes
	double posS, posD, posStiff;		// springs between nodes and ideal positions
	double colS, colD, colStiff;		// dynamic springs to avoid collisions
	vector<double> positioningStrengths;

	SolverVerlet(void) : Solver() {
		g = -700;
		hasGravity = true;
		lastTime = 0;
		velocityDamping = 0.9;
		positioningStrengths = vector<double>();
		distS = 5;	distD = 0.05;	distStiff = 1;
		posS = 1;	posD = 0.05;	posStiff = 1;
		colS = 200;	colD = 200;		colStiff = 1;
	}
	~SolverVerlet(void) {}
	vector<pair<int,Vector3d> > SolverVerlet::solveVerlet(double time, vector<SolverVerlet*>& verlets, int skID);
	void solveVerlet2 (double time, SolverData* data);
	
	virtual void solve(double time) {}

	void addSkeleton(skeleton* s, Chain* c) {
		chainSize = index2 - index1 + 1;
		int nextSK = restPositions.size();
		restPositions.push_back(vector<Vector3d>(chainSize));
		lastPositions.push_back(vector<Vector3d>(chainSize));
		currentPositions.push_back(vector<Vector3d>(chainSize));
		idealPositions.push_back(vector<Vector3d>(chainSize));
		positioningStrengths = vector<double> (chainSize, 0);

		for (int i = 0; i < chainSize; ++i) {
			currentPositions[nextSK][i] = lastPositions[nextSK][i] = restPositions[nextSK][i] = c->positions[i];
			//positioningStrengths[i] = (1 - 0.03*i);
		}
	}

	void setPositions(skeleton* s) {

		chainSize = index2 - index1 + 1;

		restPositions.resize(inputs.size());
		lastPositions.resize(inputs.size());
		currentPositions.resize(inputs.size());
		idealPositions.resize(inputs.size());
		positioningStrengths = vector<double> (chainSize, 1);

		for (int ip = 0; ip < inputs.size(); ++ip) {
			restPositions[ip].resize(chainSize);
			lastPositions[ip].resize(chainSize);
			currentPositions[ip].resize(chainSize);
			idealPositions[ip].resize(chainSize);

			for (int i = 0; i < chainSize; ++i) {
				currentPositions[0][i] = lastPositions[0][i] = restPositions[0][i] = s->joints[i]->translation;
				positioningStrengths[i] = (1 - 0.03*i);
			}
		}
	}

	void solve(SolverData* data) {
		if (hasGravity) g = data->gravity;
		else g = 0;
		time = data->time;
		fps = data->fps;
		solve();
	}

	virtual void updateDirtyness() {
		dirtyFlag = true;
		propagateDirtyness();
	}

	virtual void solve() {
		if (hasGravity) g = data->gravity;
		else g = 0;
		for (int ip = 0; ip < inputs.size(); ++ip) {
			for (int i = 0; i < chainSize; ++i) {
				idealPositions[ip][i] = inputs[ip]->positions[i + index1];
				lastPositions[ip][i] = currentPositions[ip][i];
			}
			for (int i = 0; i < inputs[ip]->positions.size(); ++i)
				outputs[ip]->positions[i] = inputs[ip]->positions[i];
		}
		int numReps = 10;
		for (int k = 0; k < numReps-1; ++k) {
			double timeInc = ((double) k / numReps) * (1 / fps);
			solve2(time + timeInc); 
		}
	}

	void solve2(double ttime) {
		// Times, times, times
		double deltaTime = ttime - lastTime;
		double timeSquared = deltaTime * deltaTime;
		lastTime = ttime;
		int relaxSteps = 5;
		int neighbourDistance = 5;

		// Solve all inputs
		for (int ip = 0; ip < inputs.size(); ++ip) {
			Chain* c = inputs[ip];

			for (int k = 0; k < relaxSteps; ++k) {

				for (int i = 0; i < currentPositions[ip].size(); ++i) {

					// NEIGHBOUR CONSTRAINTS
					for (int j = i - neighbourDistance; j < i; ++j) {
						if (j < 0) continue;

						Vector3d restDistance = (restPositions[ip][i] - restPositions[ip][j]);
						Vector3d currentDist = currentPositions[ip][i] - currentPositions[ip][j];
						double diff = currentDist.norm() - restDistance.norm();

						Vector3d delta1 = currentDist / currentDist.norm() * distS * diff;
						Vector3d delta2 = - delta1;
						Vector3d vel1 = (currentPositions[ip][i] - lastPositions[ip][i]);
						Vector3d vel2 = (currentPositions[ip][j] - lastPositions[ip][j]);
						if (j == 0) vel2 = Vector3d(0,0,0);
						double v = (vel1 - vel2).dot(currentDist.normalized());
						Vector3d damp1 = currentDist / currentDist.norm() * distD * v;
						Vector3d damp2 = - damp1;
						currentPositions[ip][i] -= (delta1+damp1)*distStiff*deltaTime;
						if (j > 0) currentPositions[ip][j] -= (delta2+damp2)*distStiff*deltaTime;
						//if (j > 0 && i < currentPositions.size()-1) currentPositions[j] -= (delta2+damp2)*(distStiff*3)*deltaTime;
					}	// end of neighbours loop

					// POSITIONING CONSTRAINTS + RIGIDNESS
					Vector3d idealPoint = idealPositions[ip][i];
					Vector3d currentPoint = currentPositions[ip][i];
					Vector3d restDistance (0,0,0);
					Vector3d currentDist = currentPositions[ip][i] - idealPoint;

					if (currentDist.norm() > 0) {			// avoid dividing by 0
						double diff = currentDist.norm() - restDistance.norm();
						Vector3d delta1 = currentDist / currentDist.norm() * posS * diff;
						Vector3d vel1 = (currentPositions[ip][i] - lastPositions[ip][i]);
						Vector3d rigid = delta1;
						double v = (currentDist.normalized()).dot(vel1.normalized());
						if (currentDist.isZero(0.001) || vel1.isZero(0.001)) v = 0;
						//double damping = ((double)(currentPositions.size()-i)) / (currentPositions.size());			
						Vector3d damp1 = currentDist / currentDist.norm() * posD * v;
						Vector3d inc = (delta1+damp1+rigid*positioningStrengths[i])*(posStiff)*deltaTime;
						currentPositions[ip][i] -= inc;
					}

				}	// end of positions loop

			}	// end of relaxing loop

			// Update points
			for (int i = 0; i < currentPositions[ip].size(); ++i) {
				Vector3d velocity = (currentPositions[ip][i] - lastPositions[ip][i]);
				velocity *= velocityDamping;
				Vector3d acceleration = Vector3d(0,g,0);
				if (i < 10) acceleration = Vector3d(0,0,0);
				if (velocity.norm() < 0.0005) velocity = Vector3d(0,0,0);
				Vector3d nextPos = currentPositions[ip][i] + velocity + acceleration * timeSquared * 0.5;
				lastPositions[ip][i] = currentPositions[ip][i];
				currentPositions[ip][i] = nextPos;
			}

		}	// end of input loop


		// Perform collisions on all of them
		for (int k = 0; k < relaxSteps; ++k) {
			for (int sk = 0; sk < inputs.size(); ++sk) {
				for (int sk2 = 0; sk2 < inputs.size(); ++sk2) {
					if (sk == sk2) continue;

					for (int i = 0; i < currentPositions[sk].size(); ++i) {

						for (int j = 0; j < currentPositions[sk2].size(); ++j) {
							double distance = (currentPositions[sk][i] - currentPositions[sk2][j]).norm();
							Vector3d currentDist = (currentPositions[sk][i] - currentPositions[sk2][j]);
							double restDistance = 50;

							if (distance < restDistance) {
								double diff = distance - restDistance;
								Vector3d delta1 = currentDist / currentDist.norm() * colS * diff;
								Vector3d vel1 = (currentPositions[sk][i] - lastPositions[sk][i]);
								double v = (vel1).dot(currentDist.normalized());
								Vector3d damp1 = currentDist / currentDist.norm() * colD * v;
								Vector3d damp2 = - damp1;
								if (i > 0) currentPositions[sk][i] -= (delta1)*colStiff*deltaTime/2;
								//if (j > 0) currentPositions[sk][j] += (delta1)*colStiff*deltaTime/2;
							}
						}	// end of neighbours
					}	// end of positions
				}	// end of input loops
			}
		}		// end of collisions
		

		for (int ip = 0; ip < inputs.size(); ++ip) {
			for (int i = 0; i < chainSize; ++i)
				outputs[ip]->positions[i + index1] = currentPositions[ip][i];
		}
	}


};

