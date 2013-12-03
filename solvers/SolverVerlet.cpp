#include "SolverVerlet.h"


void SolverVerlet::solveVerlet2 (double time, SolverData* data) {
double deltaTime = time - lastTime;
	double tsq = deltaTime * deltaTime;
	lastTime = time;

	// We love parameters
	double ks, kd, stiff, ks2, kd2, stiff2, colKS, colKD, colStiff;

	ks = 1;		kd = 0.05;		stiff = 1;
	ks2 = 1;	kd2 = 0.05;		stiff2 = 0.1;
	colKS = 1;	colKD = 0.05;	colStiff = 1;
	double dampDamping = 1.5;
	int neighbourDistance = 5;
	
	for (int k = 0; k < 30; ++k) {
		for (int i = 0; i < currentPositions.size(); ++i) {
			if (i == currentPositions.size()-1) neighbourDistance = 1;
			else neighbourDistance = 5;

			// Distance constraints: springs between nodes
			/*for (int j = i-neighbourDistance; j < i; ++j) {
				if (j < 0 || j >= currentPositions.size()) continue;

				Vector3d restDistance = (restPositions[i] - restPositions[j]);
				Vector3d currentDist = currentPositions[i] - currentPositions[j];
				double diff = currentDist.norm() - restDistance.norm();

				Vector3d delta1 = currentDist / currentDist.norm() * ks * diff;
				Vector3d delta2 = - delta1;
				Vector3d vel1 = (currentPositions[i] - lastPositions[i]);
				Vector3d vel2 = (currentPositions[j] - lastPositions[j]);
				if (j == 0) vel2 = Vector3d(0,0,0);
				double v = (vel1 - vel2).dot(currentDist.normalized());
				Vector3d damp1 = currentDist / currentDist.norm() * kd * v;
				Vector3d damp2 = - damp1;
				currentPositions[i] -= (delta1+damp1)*(stiff*3)*deltaTime;
				if (j > 0 && i < currentPositions.size()-1) currentPositions[j] -= (delta2+damp2)*stiff*deltaTime;
			}*/
			// End of distance constraints

			// Positioning constraints: these ones have proportional damping :)
			/*Vector3d idealPoint = idealPositions[i];
			Vector3d currentPoint = currentPositions[i];
			Vector3d restDistance (0,0,0);
			Vector3d currentDist = currentPositions[i] - idealPoint;

			if (currentDist.norm() > 0) {			// avoid dividing by 0
				double diff = currentDist.norm() - restDistance.norm();
				Vector3d delta1 = currentDist / currentDist.norm() * ks2 * diff;
				Vector3d vel1 = (currentPositions[i] - lastPositions[i]);
				//vel1 = Vector3d(0,0,0);
				double v = (currentDist.normalized()).dot(vel1.normalized());
				if (currentDist.isZero(0.001) || vel1.isZero(0.001)) v = 0;
				double damping = ((double)(currentPositions.size()-i)) / (currentPositions.size());

				if (i < 3) damping = 1;
				else damping = ((double)(currentPositions.size()-i+2)) / (currentPositions.size());
				
				if (i == 0) damping *= 2;
				if (i == 1) damping *= 4;
				if (i == 2) damping *= 2;
				if (i == 3) damping *= 1.17;
				if (i == 4) damping *= 1.15;
				if (i == 5) damping *= 1.05;
				if (i == 6) damping *= 1.03;
				if (i == 7) damping *= 1.01;

				//if (i >= chain.size()-2) damping = (double)2 / chain.size();
				//if (i == chain.size()-1) damping = 1;
				
				//Vector3d damp1 = currentDist / currentDist.norm() * kd2 * v;
				//Vector3d inc = (delta1+damp1)*(stiff2+damping*dampDamping	)*deltaTime;
				//currentPositions[i] -= inc;
			}	*/
			// End of "ideal point" constraints

			// Collision constraints: self-explanatory
			/*for (int sk = 0; sk < verlets.size(); ++sk) {
				if (sk == skID) continue;
				SolverVerlet* v = verlets[sk];

				Vector3d velocity = (currentPositions[i] - lastPositions[i]);
				if (velocity.norm() < 0.001 && (currentPositions[i] - idealRestPosition(i)).norm() <= 0.00005)
				//velocity = Vector3d(0,0,0);
				velocity *= velocityDamping;
				Vector3d acceleration = Vector3d(0,g,0);	
				Vector3d nextPos = currentPositions[i] + velocity + acceleration * tsq * 0.5;
				nextPos = currentPositions[i];

				for (int j = 0; j < v->chain.size(); ++j) {
					if (sk == skID && i == j) continue;
					double distance = (nextPos - v->currentPositions[j]).norm();
					Vector3d currentDist = (nextPos - v->currentPositions[j]);
					double restDistance = 50;
					//if (i == currentPositions.size() - 1) restDistance = 60;		// big head
					if (sk == skID) restDistance = (restPositions[i] - restPositions[j]).norm();

					if (distance < restDistance) {
						double diff = distance - restDistance;
						Vector3d delta1 = currentDist / currentDist.norm() * colKS * diff;
						Vector3d vel1 = (currentPositions[i] - lastPositions[i]);
						double v = (vel1).dot(currentDist.normalized());
						Vector3d damp1 = currentDist / currentDist.norm() * colKD * v;
						Vector3d damp2 = - damp1;
						currentPositions[i] -= (delta1)*colStiff*deltaTime/2;
						currentPositions[j] += (delta1)*colStiff*deltaTime/2;
					}
				}
			}*/
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
					Vector3d delta1 = currentDist / currentDist.norm() * ks2 * diff;
					Vector3d vel1 = (currentPositions[i] - lastPositions[i]);
					//vel1 = Vector3d(0,0,0);
					double v = (currentDist.normalized()).dot(vel1.normalized());
					if (currentDist.isZero(0.001) || vel1.isZero(0.001)) v = 0;
					Vector3d damp1 = currentDist / currentDist.norm() * kd2 * v;
					Vector3d inc = (delta1+damp1)*(stiff2+1	)*deltaTime;
					currentPositions[i] -= inc;
				}
			}*/
			// End of stiffness constraints
		}

		// Look constraint: this controls the last 2 nodes of the chain
			//Vector3d currentLooking = (currentPositions[lookChain.second] - currentPositions[lookChain.first]).normalized();
			//currentLooking *= (restPositions[lookChain.second] - restPositions[lookChain.first]).norm();
			
			/*Vector3d currentLooking = currentPositions[lookChain.second] - currentPositions[lookChain.first];
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
				Vector3d delta1 = currentDist / currentDist.norm() * ks2 * diff;
				Vector3d vel1 = (currentPositions[i] - lastPositions[i]);
				//vel1 = Vector3d(0,0,0);
				double v = (currentDist.normalized()).dot(vel1.normalized());
				if (currentDist.isZero(0.001) || vel1.isZero(0.001)) v = 0;
				Vector3d damp1 = currentDist / currentDist.norm() * kd2 * v;
				Vector3d inc = (delta1+damp1)*(stiff2)*deltaTime;
				currentPositions[i] -= inc;
			}*/
			// End of look
	}

	// Update points
	/*for (int i = 0; i < currentPositions.size(); ++i) {
		Vector3d velocity = (currentPositions[i] - lastPositions[i]);

		velocity *= velocityDamping;
		Vector3d acceleration = Vector3d(0,g,0);	
		if (i == 0) acceleration = Vector3d(0,0,0);

		if (velocity.norm() < 0.0005) velocity = Vector3d(0,0,0);

		Vector3d nextPos = currentPositions[i] + velocity + acceleration * tsq * 0.5;

		lastPositions[i] = currentPositions[i];
		currentPositions[i] = nextPos;
	}*/

	/*for (int i = 0; i < currentPositions.size(); ++i)
		data->currentChains[data->currentSkeletonID]->positions[i] = currentPositions[i];*/
}

