#include "SolverVerlet.h"


SolverVerlet::SolverVerlet(void)
{

	stiffness = 0.1;
	g = -70000;
	g = 0;
	velocityDamping = 0.9;
	lastTime = 0;
}


SolverVerlet::~SolverVerlet(void)
{
}

vector<pair<int,Eigen::Vector3d> > SolverVerlet::solveVerlet(double time, vector<SolverVerlet*>& verlets, int skID) {

	double deltaTime = time - lastTime;
	double tsq = deltaTime * deltaTime;
	lastTime = time;

	// We love parameters
	double ks, kd, stiff, ks2, kd2, stiff2;
	//ks = 0.1;		kd = 0.5;		stiff = 0.05;	// max: 0.5
	//ks2 = 5;		kd2 = 5;	stiff2 = 0.5;
	//ks = 0.00005;		kd = 0.000005;		stiff = 1;	// max: 0.5
	ks = 1;	kd = 0.05;	stiff = 1;
	ks2 = 0.5;	kd2 = 0.05;	stiff2 = 0.15;
	double dampDamping = 0.5;

	/*
		ks = 10;		kd = 5;		stiff = 0.5;	// max: 0.5
	ks2 = 5;	kd2 = 2;	stiff2 = 10;
	*/

	int neighbourDistance = 5;
	double colKS, colKD, colStiff;
	colKS = 5;	colKD = 5;	colStiff = 1;
	colKS = ks;
	colKD = kd;
	colStiff = stiff;

	int fixedThreshold = currentPositions.size() - 6;

	for (int k = 0; k < 20; ++k) {
		for (int i = 0; i < currentPositions.size(); ++i) {

			if (i == currentPositions.size()-1) neighbourDistance = 1;
			else neighbourDistance = 5;

			// Distance constraints: springs between nodes
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
				currentPositions[i] -= (delta1+damp1)*(stiff*3)*deltaTime;
				if (j > 0) currentPositions[j] -= (delta2+damp2)*stiff*deltaTime;
			}
			// End of distance constraints

			// Positioning constraints: these ones have proportional damping :)
			Eigen::Vector3d idealPoint = idealPositions[i];
			Eigen::Vector3d currentPoint = currentPositions[i];
			Eigen::Vector3d restDistance (0,0,0);
			Eigen::Vector3d currentDist = currentPositions[i] - idealPoint;

			//if (currentDist.norm() > 0.0005 && i < currentPositions.size()-1) {			// do not consider the "head" ones
			if (currentDist.norm() > 0.0005) {
				double diff = currentDist.norm() - restDistance.norm();
				Eigen::Vector3d delta1 = currentDist / currentDist.norm() * ks2 * diff;
				Eigen::Vector3d vel1 = (currentPositions[i] - lastPositions[i]);
				//vel1 = Vector3d(0,0,0);
				double v = (currentDist.normalized()).dot(vel1.normalized());
				if (currentDist.isZero(0.001) || vel1.isZero(0.001)) v = 0;
				double damping = ((double)(chain.size()-i)) / (chain.size());
				
				//if (i < 4) damping *= 1.25;
				//else if (i < 8) damping *= 1.15;
				if (i == chain.size()-1) damping = 1;
				
				Eigen::Vector3d damp1 = currentDist / currentDist.norm() * kd2 * v;
				Vector3d inc = (delta1+damp1)*(stiff2+damping*dampDamping	)*deltaTime;
				currentPositions[i] -= inc;
			}	
			// End of "ideal point" constraints

			// Collision constraints: self-explanatory
			for (int sk = 0; sk < verlets.size(); ++sk) {
				if (sk == skID) continue;
				SolverVerlet* v = verlets[sk];

				Eigen::Vector3d velocity = (currentPositions[i] - lastPositions[i]);
				if (velocity.norm() < 0.001 && (currentPositions[i] - idealRestPosition(i)).norm() <= 0.00005)
				//velocity = Eigen::Vector3d(0,0,0);
				velocity *= velocityDamping;
				Eigen::Vector3d acceleration = Eigen::Vector3d(0,g,0);	
				Eigen::Vector3d nextPos = currentPositions[i] + velocity + acceleration * tsq * 0.5;
				nextPos = currentPositions[i];

				for (int j = 0; j < v->chain.size(); ++j) {
					if (sk == skID && i == j) continue;
					double distance = (nextPos - v->currentPositions[j]).norm();
					Eigen::Vector3d currentDist = (nextPos - v->currentPositions[j]);
					double restDistance = 50;
					//if (i == currentPositions.size() - 1) restDistance = 60;		// big head

					if (distance < restDistance) {
						double diff = distance - restDistance;
						Eigen::Vector3d delta1 = currentDist / currentDist.norm() * colKS * diff;
						Eigen::Vector3d vel1 = (currentPositions[i] - lastPositions[i]);
						double v = (vel1).dot(currentDist.normalized());
						Eigen::Vector3d damp1 = currentDist / currentDist.norm() * colKD * v;
						Eigen::Vector3d damp2 = - damp1;
						currentPositions[i] -= (delta1)*colStiff*deltaTime/2;
						currentPositions[j] += (delta1)*colStiff*deltaTime/2;
					}
				}
			}
			// End of collision constraints

			// "Stiffness" constraints
			/*if (i >= 1 && i < 5 || (i > currentPositions.size()-4 && i < currentPositions.size()-1)) {
				if (i == 1) idealPoint = currentPositions[i-1] + Vector3d(0,1,0) * (currentPositions[i] - currentPositions[i-1]).norm();
				else Vector3d idealPoint = currentPositions[i-1] + (currentPositions[i-1] - currentPositions[i-2]).normalized() * (currentPositions[i] - currentPositions[i-1]).norm();
				Vector3d currentPoint = currentPositions[i];
				Vector3d restDistance = Vector3d(0,0,0);
				Vector3d currentDist = currentPoint - idealPoint;
				if (currentDist.norm() > 0.0005) {
					double diff = currentDist.norm() - restDistance.norm();
					Eigen::Vector3d delta1 = currentDist / currentDist.norm() * ks2 * diff;
					Eigen::Vector3d vel1 = (currentPositions[i] - lastPositions[i]);
					//vel1 = Vector3d(0,0,0);
					double v = (currentDist.normalized()).dot(vel1.normalized());
					if (currentDist.isZero(0.001) || vel1.isZero(0.001)) v = 0;
					Eigen::Vector3d damp1 = currentDist / currentDist.norm() * kd2 * v;
					Vector3d inc = (delta1+damp1)*(stiff2+1	)*deltaTime;
					currentPositions[i] -= inc;
				}
			}*/
			// End of stiffness constraints
		}

		// Look constraint: this controls the last 2 nodes of the chain
			//Vector3d currentLooking = (currentPositions[lookChain.second] - currentPositions[lookChain.first]).normalized();
			//currentLooking *= (restPositions[lookChain.second] - restPositions[lookChain.first]).norm();
			
			Vector3d currentLooking = currentPositions[lookChain.second] - currentPositions[lookChain.first];
			Vector3d desiredLooking = lookPoint - currentPositions[lookChain.first];
			Vector3d pointToLook = (desiredLooking.normalized() * currentLooking.norm());
			pointToLook += currentPositions[lookChain.first];

			Quaterniond r;	r.setFromTwoVectors(lookVector, desiredLooking);
			pointToLook = r._transformVector(currentLooking);
			pointToLook += currentPositions[lookChain.first];
			//idealPositions[currentPositions.size()-1] = pointToLook;

			//pointToLook = currentLooking.norm() * (pointToLook - currentPositions[lookChain.first]).normalized();

			int i = currentPositions.size()-1;
			Vector3d idealPoint = pointToLook;
			Vector3d currentPoint = currentPositions[lookChain.second];
			Vector3d restDistance = Vector3d(0,0,0);
			Vector3d currentDist = currentPoint - idealPoint;
			if (currentDist.norm() > 0) {
				double diff = currentDist.norm() - restDistance.norm();
				Eigen::Vector3d delta1 = currentDist / currentDist.norm() * ks2 * diff;
				Eigen::Vector3d vel1 = (currentPositions[i] - lastPositions[i]);
				//vel1 = Vector3d(0,0,0);
				double v = (currentDist.normalized()).dot(vel1.normalized());
				if (currentDist.isZero(0.001) || vel1.isZero(0.001)) v = 0;
				Eigen::Vector3d damp1 = currentDist / currentDist.norm() * kd2 * v;
				Vector3d inc = (delta1+damp1)*(stiff2)*deltaTime;
				currentPositions[i] -= inc;
			}
			// End of look
	}

	// Update points
	for (int i = 0; i < currentPositions.size(); ++i) {
		Eigen::Vector3d velocity = (currentPositions[i] - lastPositions[i]);

		velocity *= velocityDamping;
		Eigen::Vector3d acceleration = Eigen::Vector3d(0,g,0);	
		if (i == currentPositions.size()-1) acceleration = Vector3d(0,0,0);

		if (velocity.norm() < 0.0005) velocity = Vector3d(0,0,0);

		Eigen::Vector3d nextPos = currentPositions[i] + velocity + acceleration * tsq * 0.5;

		lastPositions[i] = currentPositions[i];
		currentPositions[i] = nextPos;
	}

	vector<pair<int, Eigen::Vector3d> > result(currentPositions.size());
	for (int i = 0; i < currentPositions.size(); ++i) {
		result[i] = pair<int, Eigen::Vector3d> (chain[i].first, currentPositions[i]);
	}
	return result;
}

vector<pair<int,Eigen::Vector3d> > SolverVerlet::solveCollisions(double time, vector<SolverVerlet*>& verlets, int skID) {
	double deltaTime = time - lastTime;
	double tsq = deltaTime * deltaTime;
	lastTime = time;

	// We love parameters
	double ks, kd, stiff;
	ks = 0.05;	kd = 0.005;	stiff = 1;
	double colKS, colKD, colStiff;
	colKS = 5;	colKD = 5;	colStiff = 1;
	int neighbourDistance = 2;

	for (int k = 0; k < 30; ++k) {
		for (int i = 0; i < currentPositions.size(); ++i) {

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
			// Collision constraints
			if (i == 0) continue;
			for (int sk = skID+1; sk < verlets.size(); ++sk) {
				SolverVerlet* v = verlets[sk];

				Eigen::Vector3d velocity = (currentPositions[i] - lastPositions[i]);
				if (velocity.norm() < 0.001 && (currentPositions[i] - idealRestPosition(i)).norm() <= 0.00005)
				//velocity = Eigen::Vector3d(0,0,0);
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

		if (velocity.norm() < 0.005) { // && (currentPositions[i] - idealRestPosition(i)).norm() <= 0.05) {
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
		result[i] = pair<int, Eigen::Vector3d> (chain[i].first, currentPositions[i]);
	}
	return result;
}

void SolverVerlet::setPositions() {
	restPositions.resize(chain.size());
	lastPositions.resize(chain.size());
	currentPositions.resize(chain.size());
	idealPositions.resize(chain.size());

	lastTime = 0;
	for (int i = 0; i < lastPositions.size(); ++i) {
		Eigen::Vector3d pos = chain[i].second;
		printf("Initial pos of %dth joint: %f %f %f\n", i, pos.x(), pos.y(), pos.z());
		lastPositions[i] = restPositions[i] = currentPositions[i] = idealPositions[i] = pos;
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