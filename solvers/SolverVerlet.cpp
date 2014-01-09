#include "SolverVerlet.h"
#include "AdriViewer.h"

// Constructor and destructor
SolverVerlet::SolverVerlet() : Solver() {
	g = -700;
	hasGravity = hasRigid = true;
	lastTime = 0;
	velocityDamping = 0.9;
	positioningStrengths = rigidnessStrengths = vector<double>();
	distS = 50;	distD = 0.5;	distStiff = 1;
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

		// Project onto x,y plane
		force1 = force1 - force1.dot(Vector3d(0,0,1))*Vector3d(0,0,1);
		force2 = force2 - force2.dot(Vector3d(0,0,1))*Vector3d(0,0,1);

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

		for (int ip = 0; ip < inputs.size(); ++ip) {

		}

	}

void SolverVerlet::solve2(double ttime) {
		// Times, times, times
		double deltaTime = ttime - lastTime;
		double timeSquared = deltaTime * deltaTime;
		lastTime = ttime;
		int relaxSteps = 5;
		int neighbourDistance = 2;

		clock_t start2 = clock();
		for (int ip = 0; ip < inputs.size(); ++ip) {
			Chain* c = inputs[ip];
			for (int k = 0; k < relaxSteps; ++k) {

				for (int i = 0; i < currentPositions[ip].size(); ++i) {

					// Positioning constraints
					/*if (i == currentPositions[ip].size()-2) {
						//posS = 30;
						//positioningStrengths[i] = 1;
					} else posS = 1;*/
					if (i > 15) positioningStrengths[i] = 1;
					addSpringToPoint(ip,i,0,idealPositions[ip][i],1,deltaTime,0,positioningStrengths[i]);

					// Neighbour constraints	
					for (int j = i - neighbourDistance; j < i; ++j)
						if (j >= 0) addSpringBetweenTwoJoints(ip, ip, i, j, (restPositions[ip][i] - restPositions[ip][j]).norm(), 0, deltaTime, 1, 1, 1);
					
					// Rigidness
					if (i > 15 && i < currentPositions[ip].size()-2) {
						Vector3d currentVector = (currentPositions[ip][i] - currentPositions[ip][i+1]).normalized();
						Vector3d restVector = (currentPositions[ip][i+1] - currentPositions[ip][i+2]).normalized();
						double angle = acos(currentVector.dot(restVector));
						angle = angle * 180 / M_PI;
						double threshold = 5;
						if (angle > threshold) {
							double restLength = (restPositions[ip][i] - restPositions[ip][i+1]).norm();
							addSpringToPoint(ip,i,0,currentPositions[ip][i+1] + restVector * restLength,3,deltaTime,0,1);
						}
					}

					// Collisions
					for (int sk2 = 0; sk2 < currentPositions.size(); ++sk2) {
						for (int j = 0; j < currentPositions[sk2].size(); ++j) {
							if (ip == sk2 && abs(i-j) > neighbourDistance && i < 15) {
								float radius = 20;
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

					// Rigidness
					/*if (i > 15 && i < currentPositions[ip].size()-2 && hasRigid)  {
						Vector3d currentVector = (currentPositions[ip][i] - currentPositions[ip][i+1]).normalized();
						Vector3d restVector = (currentPositions[ip][i+1] - currentPositions[ip][i+2]).normalized();
						double angle = acos(currentVector.dot(restVector));
						angle = angle * 180 / M_PI;
						double threshold = 5;
						if (angle > threshold) {
							//addSpringToPoint(ip,ip,i,i+1,(restPositions[ip][i] - restPositions[ip][i-1]).norm(),3,deltaTime,0,40,positioningStrengths[i]);
							double restLength = (restPositions[ip][i] - restPositions[ip][i+1]).norm();
							addSpringToPoint(ip,i,0,currentPositions[ip][i+1] + restVector * restLength,3,deltaTime,0,1);
							//Vector3d idealPoint = currentPositions[ip][i-1] + (restPositions[ip][i] - restPositions[ip][i-1]);
							/*double restLength = (restPositions[ip][i] - restPositions[ip][i-1]).norm();
							Vector3d idealPoint = currentPositions[ip][i-1] + restVector * restLength;
							Vector3d currentPoint = currentPositions[ip][i];
							Vector3d restDistance (0,0,0);
							Vector3d currentDist = currentPositions[ip][i] - idealPoint ;

							if (currentDist.norm() - sin(threshold*M_PI / 180)*restLength > 0) {			// avoid dividing by 0
								double diff = currentDist.norm() - sin(threshold*M_PI / 180)*restLength - restDistance.norm();
								Vector3d delta1 = currentDist / currentDist.norm() * rigidS * diff;
								Vector3d vel1 = (currentPositions[ip][i] - lastPositions[ip][i]);
								Vector3d rigid = delta1;
								double v = (currentDist.normalized()).dot(vel1.normalized());
								if (currentDist.isZero(0.001) || vel1.isZero(0.001)) v = 0;
								//double damping = ((double)(currentPositions.size()-i)) / (currentPositions.size());			
								Vector3d damp1 = currentDist / currentDist.norm() * rigidD * v;
								Vector3d inc = (delta1+damp1+rigid*positioningStrengths[i])*(rigidStiff)*deltaTime; //
								currentPositions[ip][i] -= inc;
							}
						}
					}*/

					/*Vector3d idealPoint = idealPositions[ip][i];
					Vector3d currentPoint = currentPositions[ip][i];
					Vector3d restDistance (0,0,0);
					Vector3d currentDist = currentPositions[ip][i] - idealPoint;

					if (currentDist.norm() > 0) {
						double diff = currentDist.norm() - restDistance.norm();
						Vector3d delta1 = currentDist / currentDist.norm() * posS * diff;
						Vector3d vel1 = (currentPositions[ip][i] - lastPositions[ip][i]);
						Vector3d rigid = delta1;
						double v = (currentDist.normalized()).dot(vel1.normalized());
						if (currentDist.isZero(0.001) || vel1.isZero(0.001)) v = 0;
						Vector3d damp1 = currentDist / currentDist.norm() * posD * v;
						Vector3d inc = (delta1+damp1)*positioningStrengths[i]*(posStiff)*deltaTime;
						currentPositions[ip][i] -= inc;
					}*/

					// RIGIDNESS
					/*if (i > 0 && hasRigid) {
						Vector3d currentVector = (currentPositions[ip][i] - currentPositions[ip][i-1]).normalized();
						Vector3d restVector;
						if (i == 1)		restVector = Vector3d(0,1,0);
						else			restVector = (currentPositions[ip][i-1] - currentPositions[ip][i-2]).normalized();
						double angle = acos(currentVector.dot(restVector));
						angle = angle * 180 / M_PI;
						double threshold = 10;
						if (angle > threshold) {
							//Vector3d idealPoint = currentPositions[ip][i-1] + (restPositions[ip][i] - restPositions[ip][i-1]);
							double restLength = (restPositions[ip][i] - restPositions[ip][i-1]).norm();
							Vector3d idealPoint = currentPositions[ip][i-1] + restVector * restLength;
							Vector3d currentPoint = currentPositions[ip][i];
							Vector3d restDistance (0,0,0);
							Vector3d currentDist = currentPositions[ip][i] - idealPoint ;

							if (currentDist.norm() - sin(threshold*M_PI / 180)*restLength > 0) {			// avoid dividing by 0
								double diff = currentDist.norm() - sin(threshold*M_PI / 180)*restLength - restDistance.norm();
								Vector3d delta1 = currentDist / currentDist.norm() * rigidS * diff;
								Vector3d vel1 = (currentPositions[ip][i] - lastPositions[ip][i]);
								Vector3d rigid = delta1;
								double v = (currentDist.normalized()).dot(vel1.normalized());
								if (currentDist.isZero(0.001) || vel1.isZero(0.001)) v = 0;
								//double damping = ((double)(currentPositions.size()-i)) / (currentPositions.size());			
								Vector3d damp1 = currentDist / currentDist.norm() * rigidD * v;
								Vector3d inc = (delta1+damp1+rigid*positioningStrengths[i])*(rigidStiff)*deltaTime; //
								currentPositions[ip][i] -= inc;
							}
						}
					}*/

					// COLLISIONS?
					/*int sk = ip;
					for (int sk2 = 0; sk2 < inputs.size(); ++sk2) {
						if (sk == sk2) continue;
						for (int j = 0; j < currentPositions[sk2].size(); ++j) {

							double distance = (currentPositions[sk][i] - currentPositions[sk2][j]).norm();
							Vector3d currentDist = (currentPositions[sk][i] - currentPositions[sk2][j]);
							double restDistance = 50;
							//if (i == currentPositions[sk].size() - 1) restDistance = 90;

							if (distance < restDistance) {
								double diff = distance - restDistance;
								Vector3d delta1 = currentDist / currentDist.norm() * colS * diff;
								Vector3d vel1 = (currentPositions[sk][i] - lastPositions[sk][i]);
								double v = (vel1).dot(currentDist.normalized());
								Vector3d damp1 = currentDist / currentDist.norm() * colD * v;
								Vector3d damp2 = - damp1;
								if (i > 1) currentPositions[sk][i] -= (delta1)*colStiff*deltaTime/2;
								if (j > 1) currentPositions[sk2][j] += (delta1)*colStiff*deltaTime/2;
							}
						}	
					}	// end of collisions*/
				}	// end of positions loop
			}	// end of relaxing loop

			// Update points
			for (int i = 0; i < currentPositions[ip].size(); ++i) {
				Vector3d velocity = (currentPositions[ip][i] - lastPositions[ip][i]);
				velocity *= velocityDamping;
				Vector3d acceleration = Vector3d(0,g,0);
				if (i < 1) acceleration = Vector3d(0,0,0);
				//if (i == 0) velocity = Vector3d(0,0,0);
				if (velocity.norm() < 0.0005) velocity = Vector3d(0,0,0);
				Vector3d nextPos = currentPositions[ip][i] + velocity + acceleration * timeSquared * 0.5;
				lastPositions[ip][i] = currentPositions[ip][i];
				currentPositions[ip][i] = nextPos;
			}

		}	// end of input loop
		clock_t end2 = clock();
		//printf("	Elapsed solving time: %f\n", timelapse(start2,end2));

		clock_t start = clock();

		/*// Perform collisions on all of them
		for (int k = 0; k < relaxSteps; ++k) {
			for (int sk = 0; sk < inputs.size(); ++sk) {
				for (int sk2 = sk+1; sk2 < inputs.size(); ++sk2) {
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
								if (j > 0) currentPositions[sk2][j] += (delta1)*colStiff*deltaTime/2;
							}
						}	// end of neighbours
					}	// end of positions
				}	// end of input loops
			}
		}		// end of collisions*/

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
			for (int k = 0; k < relaxSteps; ++k) {

				// Move the head to its ideal position
				int hi = currentPositions[ip].size()-1;
				//currentPositions[ip][hi] = idealPositions[ip][hi];

				Vector3d idealPoint = idealPositions[ip][hi];
				Vector3d currentPoint = currentPositions[ip][hi];
				Vector3d restDistance (0,0,0);
				Vector3d currentDist = currentPositions[ip][hi] - idealPoint;

				if (currentDist.norm() > 0) {
					double diff = currentDist.norm() - restDistance.norm();
					Vector3d delta1 = currentDist / currentDist.norm() * posS * diff;
					Vector3d vel1 = (currentPositions[ip][hi] - lastPositions[ip][hi]);
					Vector3d rigid = delta1;
					double v = (currentDist.normalized()).dot(vel1.normalized());
					if (currentDist.isZero(0.001) || vel1.isZero(0.001)) v = 0;
					Vector3d damp1 = currentDist / currentDist.norm() * posD * v;
					Vector3d inc = (delta1+damp1)*(posStiff)*deltaTime;		// +rigid*positioningStrengths[hi]
					currentPositions[ip][hi] -= inc;
				}

				for (int i = hi; i >= 0; --i) {


					// NEIGHBOUR CONSTRAINTS
					for (int j = i - neighbourDistance; j < i; ++j) {
						if (j == i || j <= 0 || j >= hi) continue;

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
						double damping = 1;
						if (i == hi) {
							damping = 1 - (1.0 / (neighbourDistance+1)) * (i - j);
							damping = 1 - ( (i - j - 1) * (1.0 / neighbourDistance) );
						}
						currentPositions[ip][j] += (delta1+damp1)*distStiff*damping*deltaTime;
					}

					// POSITIONING CONSTRAINTS
					Vector3d idealPoint = idealPositions[ip][i];
					Vector3d currentPoint = currentPositions[ip][i];
					Vector3d restDistance (0,0,0);
					Vector3d currentDist = currentPositions[ip][i] - idealPoint;

					if (currentDist.norm() > 0 && i != hi) {
						double diff = currentDist.norm() - restDistance.norm();
						Vector3d delta1 = currentDist / currentDist.norm() * posS * diff;
						Vector3d vel1 = (currentPositions[ip][i] - lastPositions[ip][i]);
						Vector3d rigid = delta1;
						double v = (currentDist.normalized()).dot(vel1.normalized());
						if (currentDist.isZero(0.001) || vel1.isZero(0.001)) v = 0;
						Vector3d damp1 = currentDist / currentDist.norm() * posD * v;
						Vector3d inc = (delta1+damp1+rigid*positioningStrengths[i])*(posStiff)*deltaTime;
						currentPositions[ip][i] -= inc;
					}

					// COLLISIONS CONSTRAINTS
					int sk = ip;
					for (int sk2 = 0; sk2 < inputs.size(); ++sk2) {
						if (sk == ip) continue;
						for (int j = 0; j < currentPositions[sk2].size(); ++j) {
							double distance = (currentPositions[sk][i] - currentPositions[sk2][j]).norm();
							Vector3d currentDist = (currentPositions[sk][i] - currentPositions[sk2][j]);
							
							double restDistance = 50;
							//if (i == hi) restDistance = 90;
							//if (i == currentPositions[sk].size()-1) restDistance = 80;

							if (distance < restDistance) {
								double diff = distance - restDistance;
								Vector3d delta1 = currentDist / currentDist.norm() * colS * diff;
								Vector3d vel1 = (currentPositions[sk][i] - lastPositions[sk][i]);
								double v = (vel1).dot(currentDist.normalized());
								Vector3d damp1 = currentDist / currentDist.norm() * colD * v;
								Vector3d damp2 = - damp1;
								if (i > 1) currentPositions[sk][i] -= (delta1)*colStiff*deltaTime/2;
								if (j > 1) currentPositions[sk2][j] += (delta1)*colStiff*deltaTime/2;
							}
						}	
					}	// end of collisions

					// RIGIDNESS
					/*if (i > 1 && hasRigid) {
						Vector3d currentVector = currentPositions[ip][i] - currentPositions[ip][i-1];
						Vector3d restVector = currentPositions[ip][i-1] - currentPositions[ip][i-2];
						double angle = acos(currentVector.dot(restVector) / (currentVector.norm() * restVector.norm()));
						angle = angle * 180 / M_PI;
						if (angle > 25) {
							//Vector3d idealPoint = currentPositions[ip][i-1] + (restPositions[ip][i] - restPositions[ip][i-1]);
							Vector3d idealPoint = currentPositions[ip][i-1] + restVector;
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
								Vector3d inc = (delta1+damp1+rigid*positioningStrengths[i])*(posStiff)*deltaTime*2;
								currentPositions[ip][i] -= inc;
							}
						}
					}*/

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
