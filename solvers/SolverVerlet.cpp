#include "SolverVerlet.h"
#include "AdriViewer.h"

// Constructor and destructor
SolverVerlet::SolverVerlet() : Solver() {
	g = -700;
	hasGravity = hasRigid = true;
	lastTime = 0;
	velocityDamping = 0.9;
	positioningStrengths = rigidnessStrengths = vector<double>();
	distS = 300;	distD = 0.5;	distStiff = 1;
	posS = 1;	posD = 0.05;	posStiff = 1;
	colS = 500;	colD = 1;		colStiff = 1;
	rigidS = 20;	rigidD = 0.05;	rigidStiff = 1;
	slider = 0;
}

SolverVerlet::~SolverVerlet() { }

void SolverVerlet::addSpringBetweenTwoJoints(int sk1, int sk2, int i1, int i2, 
											 double desiredDist, int springType, double deltaTime, int min1, int min2, double multiplier) {
	// Adds a spring between the joints sk1/i1 and sk2/i2 (sk1 and sk2 can be the same skeleton)
	// Desired dist: rest distance of the spring
	// Spring types: 0 distance (neighbors), 1 positionin (ideal pos), 2 collisions (between skeletons), 3 rigidness
	// Min1 and min2 are the minimum joints id to which we apply the force, e.g. we do not want distance applied on joint 0 (minimum = 1)
	double ks, kd, stiff;
	if (springType == 0) { ks = distS;	kd = distD;		stiff = distStiff; }
	if (springType == 1) { ks = posS;	kd = posD;		stiff = posStiff; }
	if (springType == 2) { ks = colS;	kd = colD;		stiff = colStiff; }
	if (springType == 3) { ks = rigidS;	kd = rigidD;	stiff = rigidStiff; }

	Vector3d currentDist = (currentPositions[sk1][i1] - currentPositions[sk2][i2]);
	bool applyForce = false;
	if (springType < 2) applyForce = true;
	if (springType == 2) applyForce = (currentDist.norm() < desiredDist);
	if (springType == 3) applyForce = true;

	if (applyForce) {
		double diff = currentDist.norm() - desiredDist;
		Vector3d delta1 = currentDist / currentDist.norm() * ks * diff;
		Vector3d vel1 = (currentPositions[sk1][i1] - lastPositions[sk1][i1]);
		Vector3d vel2 = (currentPositions[sk2][i2] - lastPositions[sk2][i2]);
		double v1 = (vel1).dot(currentDist.normalized());
		double v2 = (vel2).dot(currentDist.normalized());
		Vector3d damp1 = currentDist / currentDist.norm() * kd * v1;
		Vector3d damp2 = currentDist / currentDist.norm() * kd * v2;
		Vector3d force1 = delta1 + damp1;
		Vector3d force2 = delta1 + damp2;

		
		if (springType == 3) force1 += force1*multiplier;
		if (springType == 3) force2 += force2*multiplier;

		//if (springType != 2) {
			// Calculate the "flexion plane" normal
			Vector3d normal1 = Vector3d(0,1,0);
			normal1 = data->skeletons[sk1]->joints[i1]->rotation._transformVector(data->skeletons[sk1]->joints[i1]->restRot.inverse()._transformVector(normal1));
			Vector3d normal2 = Vector3d(0,1,0);
			normal2 = data->skeletons[sk2]->joints[i2]->rotation._transformVector(data->skeletons[sk2]->joints[i2]->restRot.inverse()._transformVector(normal2));

			// Project onto x,y plane
			normal1 = normal2 = Vector3d(0,0,1);
			force1 = force1 - force1.dot(normal1)*normal1;
			force2 = force2 - force2.dot(normal2)*normal2;
		//}

		if (i1 >= min1) currentPositions[sk1][i1] -= force1*stiff*deltaTime/2;
		if (i2 >= min2) currentPositions[sk2][i2] += force2*stiff*deltaTime/2;
	}
}

void SolverVerlet::addSpringToPoint (int sk1, int i1, double desiredDist, Vector3d p, int springType, double deltaTime, int min1, double multiplier) {
double ks, kd, stiff;
	if (springType == 0) { ks = distS;	kd = distD;		stiff = distStiff; }
	if (springType == 1) { ks = posS;	kd = posD;		stiff = posStiff; }
	if (springType == 2) { ks = colS;	kd = colD;		stiff = colStiff; }
	if (springType == 3) { ks = rigidS;	kd = rigidD;	stiff = rigidStiff; }

	Vector3d currentDist = (currentPositions[sk1][i1] - p);
	bool applyForce = currentDist.norm() > desiredDist;

	if (applyForce) {
		double diff = currentDist.norm() - desiredDist;
		Vector3d delta1 = currentDist / currentDist.norm() * ks * diff;
		Vector3d vel1 = (currentPositions[sk1][i1] - lastPositions[sk1][i1]);
		double v1 = (vel1).dot(currentDist.normalized());
		Vector3d damp1 = currentDist / currentDist.norm() * kd * v1;
		Vector3d force1 = (delta1+damp1);

		if (springType != 1) {
			// Calculate the "flexion plane" normal
			Vector3d normal1 = Vector3d(0,1,0);
			normal1 = data->skeletons[sk1]->joints[i1]->rotation._transformVector(data->skeletons[sk1]->joints[i1]->restRot.inverse()._transformVector(normal1));

			// Project onto x,y plane
			normal1 = Vector3d(0,0,1);
			force1 = force1 - force1.dot(normal1)*normal1;
		}
		
		if (springType == 3) {
			force1 += force1*multiplier;
			if (i1 >= min1) currentPositions[sk1][i1] -= (force1)*stiff*deltaTime;
		} else if (i1 >= min1) currentPositions[sk1][i1] -= (force1)*stiff*multiplier*deltaTime;
	}
}

void SolverVerlet::solve() {
	if (hasGravity) g = data->gravity;
	else g = 0;
	hasRigid = data->rigidness;

	for (int ip = 0; ip < inputs.size(); ++ip) {
		for (int i = 0; i < chainSize; ++i) {
			idealPositions[ip][i] = inputs[ip]->positions[i + index1];
			if (i == chainSize-2) idealPositions[ip][i] += Vector3d(0,slider,0);
			lastPositions[ip][i] = currentPositions[ip][i];
		}

		for (int i = 0; i < inputs[ip]->positions.size(); ++i)
			outputs[ip]->positions[i] = inputs[ip]->positions[i];

	}

	int numReps = 10;
	for (int k = 0; k < numReps-1; ++k) {
		double timeInc = ((double) k / numReps) * (1 / fps);
		if (!lookSolver)	solve2(time + timeInc); 
		else				solve3(time + timeInc);
	}
}

void SolverVerlet::solve2(double ttime) {
		// Times, times, times
		double deltaTime = ttime - lastTime;
		double timeSquared = deltaTime * deltaTime;
		lastTime = ttime;
		int relaxSteps = 15;
		int neighbourDistance = 2;

		clock_t start2 = clock();
		for (int ip = 0; ip < inputs.size(); ++ip) {
			Chain* c = inputs[ip];
			for (int k = 0; k < relaxSteps; ++k) {

				for (int i = 0; i < currentPositions[ip].size(); ++i) {

					/*if (i == currentPositions[ip].size()-2) {
						//posS = 30;
						//positioningStrengths[i] = 1;
					} else posS = 1;*/

					// Positioning constraints
					//if (i < 2) posS = 30;
					//else posS = 1;
					//if (i > 15) positioningStrengths[i] = 0.25;
					//if (i < 2) currentPositions[ip][i] = idealPositions[ip][i];
					//else addSpringToPoint(ip,i,0,idealPositions[ip][i],1,deltaTime,0,positioningStrengths[i]);
					addSpringToPoint(ip,i,0,idealPositions[ip][i],1,deltaTime,0,positioningStrengths[i]);
					
					// Top Rigidness
					/*if (i > 13 && i < currentPositions[ip].size()-2) {
						Vector3d currentVector = (currentPositions[ip][i] - currentPositions[ip][i+1]).normalized();
						Vector3d restVector = (currentPositions[ip][i+1] - currentPositions[ip][i+2]).normalized();
						double angle = acos(currentVector.dot(restVector));
						angle = angle * 180 / M_PI;
						double threshold = 1;
						if (angle > threshold) {
							double restLength = (restPositions[ip][i] - restPositions[ip][i+1]).norm();
							addSpringToPoint(ip,i,0,currentPositions[ip][i+1] + restVector * restLength,3,deltaTime,0,1);
						}
					}*/

					// Bot rigidness
					/*if (i > 0 && i < 2) {
						Vector3d currentVector = (currentPositions[ip][i] - currentPositions[ip][i-1]).normalized();
						Vector3d restVector;
						if (i == 1) restVector = Vector3d(0,1,0);
						else restVector = (currentPositions[ip][i-1] - currentPositions[ip][i-2]).normalized();
						double angle = acos(currentVector.dot(restVector));
						angle = angle * 180 / M_PI;
						double threshold = 0;
						if (angle > threshold) {
							double restLength = (restPositions[ip][i] - restPositions[ip][i-1]).norm();
							addSpringToPoint(ip,i,0,currentPositions[ip][i-1] + restVector * restLength,3,deltaTime,0,1);
						}
					}*/

					// Neighbour constraints	
					if (i >= currentPositions[ip].size()-2) neighbourDistance = 5;
					else neighbourDistance = 2;
					for (int j = i - neighbourDistance; j < i; ++j)
						if (j >= 0) addSpringBetweenTwoJoints(ip, ip, i, j, (restPositions[ip][i] - restPositions[ip][j]).norm(), 0, deltaTime, 1, 2, 1);

					// Distance to vertical constraints
					/*if ((idealPositions[ip][i] - currentPositions[ip][i]).norm() > 30) {
						Vector3d p = currentPositions[ip][i];
						p.x() = idealPositions[ip][i].x();
						addSpringToPoint(ip,i,0,p,1,deltaTime,0,1);
					}*/

					// Collisions
					for (int sk2 = 0; sk2 < currentPositions.size(); ++sk2) {
						for (int j = 0; j < currentPositions[sk2].size(); ++j) {
							if (ip == sk2 && abs(i-j) > neighbourDistance && i < 15) {
								float radius = 12;
								addSpringBetweenTwoJoints(ip,sk2,i,j,radius*2,2,deltaTime,1,1,1);
							}
						}
					}

					// Collision with "ground"
					if (i > 0 && currentPositions[ip][i].y() < 0) {
						Vector3d p = currentPositions[ip][i];
						p.y() = 0;
						addSpringToPoint(ip,i,0,p,0,deltaTime,1,1);
					}
				}	// end of positions loop
			}	// end of relaxing loop

			// Update points
			for (int i = 0; i < currentPositions[ip].size(); ++i) {
				Vector3d velocity = (currentPositions[ip][i] - lastPositions[ip][i]);
				velocity *= velocityDamping;
				Vector3d acceleration = Vector3d(0,g,0);
				if (i < 2 || i == currentPositions[ip].size()-1) acceleration = Vector3d(0,0,0);
				if (velocity.norm() < 0.0005) velocity = Vector3d(0,0,0);
				Vector3d nextPos = currentPositions[ip][i] + velocity + acceleration * timeSquared * 0.5;
				lastPositions[ip][i] = currentPositions[ip][i];
				currentPositions[ip][i] = nextPos;
			}

		}	// end of input loop
		clock_t end2 = clock();
		//printf("	Elapsed solving time: %f\n", timelapse(start2,end2));

		clock_t start = clock();

		clock_t end = clock();
		//printf("	Elapsed collision time: %f\n", timelapse(start,end));

		for (int ip = 0; ip < inputs.size(); ++ip) {
			for (int i = 0; i < chainSize; ++i)
				outputs[ip]->positions[i + index1] = currentPositions[ip][i];
		}
	}

void SolverVerlet::solve3(double ttime) {
		// Times, times, times
		double deltaTime = ttime - lastTime;
		double timeSquared = deltaTime * deltaTime;
		lastTime = ttime;
		int relaxSteps = 5;
		int neighbourDistance = 5;

		

		// Solve all inputs
		for (int ip = 0; ip < inputs.size(); ++ip) {
			Chain* c = inputs[ip];

			Vector3d look = idealPositions[ip][idealPositions[ip].size()-1] - idealPositions[ip][idealPositions[ip].size()-2];

			for (int k = 0; k < relaxSteps; ++k) {

				for (int i = 0; i < currentPositions[ip].size(); ++i) {

					/*if (i == currentPositions[ip].size()-2) {
						//posS = 30;
						//positioningStrengths[i] = 1;
					} else posS = 1;*/

					// Positioning constraints
					addSpringToPoint(ip,i,0,idealPositions[ip][i],1,deltaTime,0,positioningStrengths[i]);

					// Rigidness
					if (i > 15 && i < currentPositions[ip].size()-2) {
						Vector3d currentVector = (currentPositions[ip][i] - currentPositions[ip][i+1]).normalized();
						Vector3d restVector = (currentPositions[ip][i+1] - currentPositions[ip][i+2]).normalized();
						double angle = acos(currentVector.dot(restVector));
						angle = angle * 180 / M_PI;
						double threshold = 5;
						if (angle > threshold) {
							double restLength = (restPositions[ip][i] - restPositions[ip][i+1]).norm();
							addSpringToPoint(ip,i,0,currentPositions[ip][i+1] + restVector * restLength,0,deltaTime,0,1);
						}
					}

					// Neighbour constraints	
					for (int j = i - neighbourDistance; j < i; ++j)
						if (j >= 0 && i < currentPositions[ip].size()-1) addSpringBetweenTwoJoints(ip, ip, i, j, (restPositions[ip][i] - restPositions[ip][j]).norm(), 0, deltaTime, 1, 1, 1);

					// Collisions
					for (int sk2 = 0; sk2 < currentPositions.size(); ++sk2) {
						for (int j = 0; j < currentPositions[sk2].size(); ++j) {
							if (ip == sk2 && abs(i-j) > neighbourDistance && i < 15) {
								float radius = 15;
								addSpringBetweenTwoJoints(ip,sk2,i,j,radius*2,2,deltaTime,1,1,1);
							}
						}
					}

					// Collision with "ground"
					if (i > 0 && currentPositions[ip][i].y() < 0) {
						Vector3d p = currentPositions[ip][i];
						p.y() = 0;
						addSpringToPoint(ip,i,0,p,0,deltaTime,1,1);
					}
				}	// end of positions loop
			}	// end of relaxing loop

			// Update points
			for (int i = 0; i < currentPositions[ip].size(); ++i) {
				Vector3d velocity = (currentPositions[ip][i] - lastPositions[ip][i]);
				velocity *= velocityDamping;
				Vector3d acceleration = Vector3d(0,g,0);
				if (i < 2) acceleration = Vector3d(0,0,0);
				//if (i == 0) velocity = Vector3d(0,0,0);
				if (velocity.norm() < 0.0005) velocity = Vector3d(0,0,0);
				Vector3d nextPos = currentPositions[ip][i] + velocity + acceleration * timeSquared * 0.5;
				lastPositions[ip][i] = currentPositions[ip][i];
				currentPositions[ip][i] = nextPos;
			}

		}	// end of input loop

		glDisable(GL_LIGHTING);
		/*// Perform collisions on all of them
		for (int k = 0; k < relaxSteps; ++k) {

			for (int sk = 0; sk < inputs.size(); ++sk) {
				for (int sk2 = sk+1; sk2 < inputs.size(); ++sk2) {
					for (int i = 0; i < currentPositions[sk].size(); i += 1) {
						for (int j = 0; j < currentPositions[sk2].size(); j += 1) {
							double distance = (currentPositions[sk][i] - currentPositions[sk2][j]).norm();
							Vector3d currentDist = (currentPositions[sk][i] - currentPositions[sk2][j]);
							double restDistance = 50;

							if (distance < restDistance) {
								double diff = distance - restDistance;
								Vector3d delta1 = currentDist / currentDist.norm() * colS * diff;
								Vector3d delta2 = delta1;
								Vector3d vel1 = (currentPositions[sk][i] - lastPositions[sk][i]);
								Vector3d vel2 = (currentPositions[sk2][j] - lastPositions[sk2][j]);
								Vector3d damp1 = currentDist / currentDist.norm() * colD * (vel1).dot(currentDist.normalized());
								Vector3d damp2 = currentDist / currentDist.norm() * colD * (vel2).dot(currentDist.normalized());
								if (i > 0) currentPositions[sk][i] -= (delta1+damp1)*colStiff*deltaTime/2;
								if (j > 0) currentPositions[sk2][j] += (delta2+damp2)*colStiff*deltaTime/2;
							}
						}	// end of neighbours
					}	// end of positions
				}	// end of input loops
			}	// end of relax steps
		}	// end of collisions*/

		glEnable(GL_LIGHTING);

		for (int ip = 0; ip < inputs.size(); ++ip) {
			for (int i = 0; i < chainSize; ++i)
				outputs[ip]->positions[i + index1] = currentPositions[ip][i];
		}
	}
