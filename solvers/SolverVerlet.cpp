#include "SolverVerlet.h"


SolverVerlet::SolverVerlet(void)
{

	stiffness = 1;
	g = -9.8;
	g = 0;
	velocityDamping = 0.5;
	lastTime = 0;
}


SolverVerlet::~SolverVerlet(void)
{
}

void SolverVerlet::bake(int maxFrames, double deltaTimePerFrame) {
	bakedPositions.resize(maxFrames);
	deltaTimePerFrame /= 1;

	for (int frame = 0; frame < bakedPositions.size(); ++frame) {

		bakedPositions[frame] = (vector<Eigen::Vector3d> (currentPositions.size()));



	}
}

/*
vector<pair<int,Eigen::Vector3d> > SolverVerlet::solve(double time) {
	vector<pair<int,Eigen::Vector3d> > result(chain.size());
	double deltaTime = (time - lastTime);

	// First, solve constraints (distances between links) ( should be repeated k times )
	for (int i = 0; i < restPositions.size()-1; ++i) {
		Eigen::Vector3d restDistance = restPositions[i+1] - restPositions[i];
		Eigen::Vector3d currentDist = currentPositions[i+1] - currentPositions[i];
		double rd = restDistance.norm();
		double cd = currentDist.norm();
		double diff = (rd - cd) / rd;
		double scalarP1 = 0.5 * stiffness;
		double scalarP2 = stiffness - scalarP1;
		Eigen::Vector3d currentI = currentDist * scalarP1 * diff;
		currentPositions[i] -= currentDist * scalarP1 * diff * deltaTime;
		currentPositions[i+1] += currentDist * scalarP2 * diff * deltaTime;
	}

	double tsq = deltaTime*deltaTime;

	// Then update physics of each point
	for (int i = 0; i < chain.size(); ++i) {
		Eigen::Vector3d velocity = currentPositions[i] - lastPositions[i];						// velocity = inertia
		Eigen::Vector3d nextPos = currentPositions[i] + velocity + Eigen::Vector3d(0,-g,0) * tsq;		// apply gravity

		if (i > 0) {
			printf("Current pos of 2nd joint: %f %f %f\n", nextPos.X(), nextPos.Y(), nextPos.Z());
			
			// Which angle should we rotate the previous joint in order to position this one in nextPos?
			double tanX = currentPositions[i-1].X() - lastPositions[i-1].X();
			double tanY = currentPositions[i-1].Y() - lastPositions[i-1].Y();
			double tanZ = currentPositions[i-1].Z() - lastPositions[i-1].Z();

			double angleX = atan(tanX);
			double angleY = atan(tanY);
			double angleZ = atan(tanZ);

			result[i-1] = pair<int, Eigen::Vector3d> (chain[i-1].second, Eigen::Vector3d(angleX, angleY, angleZ));
			Eigen::Vector3d finalRot = result[i-1].second * 180 / 3.141592;
			//result[i-1].second = finalRot - chain[i-1].first->rot;
		}

		lastPositions[i] = currentPositions[i];
		currentPositions[i] = nextPos;

	}

	lastTime = time;
	result[result.size()-1] = pair<int, Eigen::Vector3d> (chain[result.size()-1].second, Eigen::Vector3d(0,0,0));
	return result;
}
*/

vector<pair<int,Eigen::Vector3d> > SolverVerlet::solveVerlet(double time, vector<SolverVerlet*>& verlets, int skID) {

	double deltaTime = time - lastTime;
	double tsq = deltaTime * deltaTime;
	lastTime = time;

	// We love parameters
	double ks, kd, stiff, ks2, kd2, stiff2;
	ks = 5;		kd = 5;		stiff = 0.1;	// max: 0.5
	ks2 = 0.5;	kd2 = 2;	stiff2 = 10;
	int neighbourDistance = 3;
	double colKS, colKD, colStiff;
	colKS = 5;	colKD = 5;	colStiff = 1;

	for (int k = 0; k < 300; ++k) {
		for (int i = 0; i < currentPositions.size(); ++i) {

			if (i < 10) neighbourDistance = 10;
			else neighbourDistance = 3;
			// Distance constraints
			for (int j = i-neighbourDistance; j < i; ++j) {
				if (j < 0) continue;
				Eigen::Vector3d restDistance = (restPositions[i] - restPositions[j]);
				Eigen::Vector3d currentDist = currentPositions[i] - currentPositions[j];
				double diff = currentDist.norm() - restDistance.norm();

				Eigen::Vector3d delta1 = currentDist / currentDist.norm() * ks * diff;
				Eigen::Vector3d delta2 = - delta1;
				Eigen::Vector3d vel1 = (currentPositions[i] - lastPositions[i]);
				Eigen::Vector3d vel2 = (currentPositions[j] - lastPositions[j]);
				if (j == 0) vel2 = Eigen::Vector3d(0,0,0);
				double v = (vel1 - vel2).dot(currentDist.normalized());
				Eigen::Vector3d damp1 = currentDist / currentDist.norm() * kd * v;
				Eigen::Vector3d damp2 = - damp1;
				currentPositions[i] -= (delta1+damp1)*stiff*deltaTime;
				if (j > 0) currentPositions[j] -= (delta2+damp2)*stiff*deltaTime;
			} 
			// End of distance constraints

			// "Ideal point" constraints, they try to maintain the angle
			/*if (i > 0) {
				Eigen::Vector3d idealPoint = idealRestPosition(i);
				Eigen::Vector3d restDistance (0,0,0);
				Eigen::Vector3d currentDist = currentPositions[i] - idealPoint;

				if (currentDist.norm() > 1) {
					double diff = currentDist.norm() - restDistance.norm();
					Eigen::Vector3d delta1 = currentDist / currentDist.norm() * ks2 * diff;
					Eigen::Vector3d vel1 = (currentPositions[i] - lastPositions[i]);
					double v = (vel1).dot(currentDist.normalized());
					Eigen::Vector3d damp1 = currentDist / currentDist.norm() * kd2 * v;
					currentPositions[i] -= (delta1+damp1)*stiff2*deltaTime;
				}	
			}*/
			// End of "ideal point" constraints

			// Collision constraints
			if (i == 0) continue;
			for (int sk = skID+1; sk < verlets.size(); ++sk) {
				SolverVerlet* v = verlets[sk];

				Eigen::Vector3d velocity = (currentPositions[i] - lastPositions[i]);
				if (velocity.norm() < 0.001 && (currentPositions[i] - idealRestPosition(i)).norm() <= 0.00005)
				velocity = Eigen::Vector3d(0,0,0);
				velocity *= velocityDamping;
				Eigen::Vector3d acceleration = Eigen::Vector3d(0,g,0);	
				Eigen::Vector3d nextPos = currentPositions[i] + velocity + acceleration * tsq * 0.5;
				nextPos = currentPositions[i];

				for (int j = 0; j < v->chain.size(); ++j) {
					if (sk == skID && i == j) continue;
					double distance = (nextPos - v->currentPositions[j]).norm();
					Eigen::Vector3d currentDist = (nextPos - v->currentPositions[j]);
					if (distance < 50) {
						double restDistance = 50;
						double diff = distance - restDistance;
						Eigen::Vector3d delta1 = currentDist / currentDist.norm() * colKS * diff;
						Eigen::Vector3d vel1 = (currentPositions[i] - lastPositions[i]);
						double v = (vel1).dot(currentDist.normalized());
						Eigen::Vector3d damp1 = currentDist / currentDist.norm() * colKD * v;
						Eigen::Vector3d damp2 = - damp1;
						currentPositions[i] -= (delta1+damp1)*colStiff*deltaTime;
					}
				}
			}
			// End of collision constraints
		}
	}

	// Update points
	for (int i = 1; i < currentPositions.size(); ++i) {
		Eigen::Vector3d velocity = (currentPositions[i] - lastPositions[i]);

		if (velocity.norm() < 1) { // && (currentPositions[i] - idealRestPosition(i)).norm() <= 0.05) {
			velocity = Eigen::Vector3d(0,0,0);
			currentPositions[i] = lastPositions[i];
			continue;
		}
		//else printf("Velocity norm: %f\n", velocity.norm());

		velocity *= velocityDamping;
		Eigen::Vector3d acceleration = Eigen::Vector3d(0,g,0);	
		Eigen::Vector3d nextPos = currentPositions[i] + velocity + acceleration * tsq * 0.5;

		lastPositions[i] = currentPositions[i];
		currentPositions[i] = nextPos;
	}

	vector<pair<int, Eigen::Vector3d> > result(currentPositions.size());
	for (int i = 0; i < currentPositions.size(); ++i) {
		result[i] = pair<int, Eigen::Vector3d> (chain[i].second, currentPositions[i]);
	}
	return result;
	//currentPositions[0] = Eigen::Vector3d(xvalue,0,0);
}

void SolverVerlet::setPositions() {
	restPositions.resize(chain.size());
	lastPositions.resize(chain.size());
	currentPositions.resize(chain.size());

	lastTime = 0;
	for (int i = 0; i < lastPositions.size(); ++i) {
		Eigen::Vector3d pos = chain[i].first->worldPosition;
		printf("Initial pos of %dth joint: %f %f %f\n", i, pos.x(), pos.y(), pos.z());
		lastPositions[i] = restPositions[i] = currentPositions[i] = pos;
	}
}

Eigen::Vector3d SolverVerlet::idealRestPosition(int i) {
	if (i == 0) assert(false);
	//return restPositions[i] - restPositions[i-1] + currentPositions[i-1];
	
	int depth = 1;
	Eigen::Vector3d position = currentPositions[i-depth];
	for (int j = i-depth; j < i; ++j) {
		Eigen::Vector3d deltaPos = restPositions[j+1] - restPositions[j];
		position = position + deltaPos;
	}
	return position;
}