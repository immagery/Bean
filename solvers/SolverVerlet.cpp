#include "SolverVerlet.h"


SolverVerlet::SolverVerlet(void)
{

	stiffness = 1;
	g = -9.8;
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

		bakedPositions[frame] = (vector<Point3d> (currentPositions.size()));



	}
}

/*
vector<pair<int,Point3d> > SolverVerlet::solve(double time) {
	vector<pair<int,Point3d> > result(chain.size());
	double deltaTime = (time - lastTime);

	// First, solve constraints (distances between links) ( should be repeated k times )
	for (int i = 0; i < restPositions.size()-1; ++i) {
		Point3d restDistance = restPositions[i+1] - restPositions[i];
		Point3d currentDist = currentPositions[i+1] - currentPositions[i];
		double rd = restDistance.Norm();
		double cd = currentDist.Norm();
		double diff = (rd - cd) / rd;
		double scalarP1 = 0.5 * stiffness;
		double scalarP2 = stiffness - scalarP1;
		Point3d currentI = currentDist * scalarP1 * diff;
		currentPositions[i] -= currentDist * scalarP1 * diff * deltaTime;
		currentPositions[i+1] += currentDist * scalarP2 * diff * deltaTime;
	}

	double tsq = deltaTime*deltaTime;

	// Then update physics of each point
	for (int i = 0; i < chain.size(); ++i) {
		Point3d velocity = currentPositions[i] - lastPositions[i];						// velocity = inertia
		Point3d nextPos = currentPositions[i] + velocity + Point3d(0,-g,0) * tsq;		// apply gravity

		if (i > 0) {
			printf("Current pos of 2nd joint: %f %f %f\n", nextPos.X(), nextPos.Y(), nextPos.Z());
			
			// Which angle should we rotate the previous joint in order to position this one in nextPos?
			double tanX = currentPositions[i-1].X() - lastPositions[i-1].X();
			double tanY = currentPositions[i-1].Y() - lastPositions[i-1].Y();
			double tanZ = currentPositions[i-1].Z() - lastPositions[i-1].Z();

			double angleX = atan(tanX);
			double angleY = atan(tanY);
			double angleZ = atan(tanZ);

			result[i-1] = pair<int, Point3d> (chain[i-1].second, Point3d(angleX, angleY, angleZ));
			Point3d finalRot = result[i-1].second * 180 / 3.141592;
			//result[i-1].second = finalRot - chain[i-1].first->rot;
		}

		lastPositions[i] = currentPositions[i];
		currentPositions[i] = nextPos;

	}

	lastTime = time;
	result[result.size()-1] = pair<int, Point3d> (chain[result.size()-1].second, Point3d(0,0,0));
	return result;
}
*/

vector<pair<int,Point3d> > SolverVerlet::solve(double time) {

	double deltaTime = time - lastTime;
	double tsq = deltaTime * deltaTime;
	lastTime = time;

	// Get the new current positions

	// Update points
	for (int i = 1; i < currentPositions.size(); ++i) {
		Point3d velocity = (currentPositions[i] - lastPositions[i]);

		if (velocity.Norm() < 0.001 && (currentPositions[i] - idealRestPosition(i)).Norm() <= 0.00005) {
			velocity = Point3d(0,0,0);
		}

		velocity *= velocityDamping;
		Point3d acceleration = Point3d(0,g,0);	
		Point3d nextPos = currentPositions[i] + velocity + acceleration * tsq * 0.5;



		lastPositions[i] = currentPositions[i];
		currentPositions[i] = nextPos;
	}

	// We love parameters
	double ks, kd, stiff, ks2, kd2, stiff2;
	ks = 5;		kd = 5;		stiff = 0.1;	// max: 0.5
	ks2 = 0.5;	kd2 = 2;	stiff2 = 10;
	int neighbourDistance = 10;

	for (int k = 0; k < 100; ++k) {
		for (int i = 0; i < currentPositions.size(); ++i) {
			// Distance constraints
			for (int j = i-neighbourDistance; j < i; ++j) {
				if (j < 0) continue;
				Point3d restDistance = (restPositions[i] - restPositions[j]);
				Point3d currentDist = currentPositions[i] - currentPositions[j];
				double diff = currentDist.Norm() - restDistance.Norm();

				Point3d delta1 = currentDist / currentDist.Norm() * ks * diff;
				Point3d delta2 = - delta1;
				Point3d vel1 = (currentPositions[i] - lastPositions[i]);
				Point3d vel2 = (currentPositions[j] - lastPositions[j]);
				if (j == 0) vel2 = Point3d(0,0,0);
				double v = (vel1 - vel2).dot(currentDist.normalized());
				Point3d damp1 = currentDist / currentDist.Norm() * kd * v;
				Point3d damp2 = - damp1;
				currentPositions[i] -= (delta1+damp1)*stiff*deltaTime;
				if (j > 0) currentPositions[j] -= (delta2+damp2)*stiff*deltaTime;
			} 
			// End of distance constraints

			// "Ideal point" constraints, they try to maintain the angle
			if (i > 0) {
				Point3d idealPoint = idealRestPosition(i);
				Point3d restDistance (0,0,0);
				Point3d currentDist = currentPositions[i] - idealPoint;

				if (currentDist.Norm() > 1) {
					double diff = currentDist.Norm() - restDistance.Norm();
					Point3d delta1 = currentDist / currentDist.Norm() * ks2 * diff;
					Point3d vel1 = (currentPositions[i] - lastPositions[i]);
					double v = (vel1).dot(currentDist.normalized());
					Point3d damp1 = currentDist / currentDist.Norm() * kd2 * v;
					currentPositions[i] -= (delta1+damp1)*stiff2*deltaTime;
				}	
			}
		}
	}

	vector<pair<int,Point3d> > result(currentPositions.size());
	for (int i = 0; i < currentPositions.size(); ++i) {
		result[i] = pair<int, Point3d> (chain[i].second, currentPositions[i]);
	}
	return result;
	//currentPositions[0] = Point3d(xvalue,0,0);
}


void SolverVerlet::setPositions() {
	restPositions.resize(chain.size());
	lastPositions.resize(chain.size());
	currentPositions.resize(chain.size());
	lastFramePositions.resize(chain.size());

	lastTime = 0;
	for (int i = 0; i < lastPositions.size(); ++i) {
		Point3d pos = chain[i].first->worldPosition;
		printf("Initial pos of %dth joint: %f %f %f\n", i, pos.X(), pos.Y(), pos.Z());
		lastPositions[i] = restPositions[i] = currentPositions[i] = pos;
	}
}

Point3d SolverVerlet::idealRestPosition(int i) {
	if (i == 0) assert(false);
	return restPositions[i] - restPositions[i-1] + currentPositions[i-1];
	
	/*Point3d position = currentPositions[0];
	for (int j = 0; j < i; ++j) {
		Point3d deltaPos = restPositions[j+1] - restPositions[j];
		position = position + deltaPos;
	}
	return position;*/
}