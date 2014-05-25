#include "SolverVerlet.h"
#include "AdriViewer.h"

// Constructor and destructor
SolverVerlet::SolverVerlet(int _id) : Solver(_id) {
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
	nextVerlet = NULL;
	printStuff = false;
	desvinculado = 20;
	minVinculado = 0;
	attackFlag = false;
	moving = false;
	lookPoint = Vector3d(0,600,0);
}

SolverVerlet::~SolverVerlet() { }

void SolverVerlet::disableAttack() {
	if (!attackFlag) return;
	attackFlag = false;
	attackCurves[0].clear();
	for (int i = 0; i < currentPositions[0].size(); ++i) {
		nextVerlet->currentPositions[0][i] = currentPositions[0][i];
		nextVerlet->lastPositions[0][i] = currentPositions[0][i];
		
	}
}

void SolverVerlet::buildAttackCurves() {
	attackFlag = true;
	for (int ip = 0; ip < inputs.size(); ++ip) {
		attackCurves[ip].clear();
		for (int i = 0; i < idealPositions[ip].size(); ++i) {
			attackCurves[ip].push_back(idealPositions[ip][i]);
		}
	}

	/*attackCurve.clear();
	attackCurve = vector<Vector3d>();
	for (int i = 0; i < idealPositions[0].size(); ++i) {
		attackCurve.push_back(idealPositions[0][i]);
		currentPositions[0][i] = lastPositions[0][i] = idealPositions[0][i];
	}*/
}

void SolverVerlet::scaleCurve() {
	for (int ip = 0; ip < inputs.size(); ++ip) {
		Vector3d axis = idealPositions[ip][0] - idealPositions[ip][idealPositions[ip].size()-1];
		Quaterniond q;	q.setFromTwoVectors(axis.normalized(),Vector3d(1,0,0));
		for (int i = 5; i < idealPositions[ip].size(); ++i) {
			Vector3d v = idealPositions[ip][i] - idealPositions[ip][idealPositions[ip].size()-1];
			v = q._transformVector(v);
			v.x() *= 1.01;

			idealPositions[ip][i] = idealPositions[ip][idealPositions[ip].size()-1] + q.inverse()._transformVector(v);
		}
	}
}

void SolverVerlet::placeAttackNode(int sk) {
	int headIndex = currentPositions[sk].size()-1;
	Vector3d lastPos = currentPositions[sk][headIndex];
	Vector3d previousPos = currentPositions[sk][headIndex-1];
	attackCurves[sk].push_back(currentPositions[sk][headIndex]);
	attackCurves[sk][attackCurves[sk].size()-2] = previousPos + (lastPos - previousPos)*0.5;
	//attackCurves[sk].push_back(curves[sk][curves[sk].size()-1]);
	curves[sk] = attackCurves[sk];
}

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

void SolverVerlet::addSpringBetweenTwoJoints3D(int sk1, int sk2, int i1, int i2, 
											 double desiredDist, int springType, double deltaTime, int min1, int min2, double multiplier) {
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

		
		if (springType == 3) {
			force1 += force1*multiplier;
			force2 += force2*multiplier;
		} else if (springType == 0) {
			force1 *= multiplier;
			force2 *= multiplier;
		}

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

	if (attackFlag) 
	{
		for (int ip = 0; ip < inputs.size(); ++ip) 
		{
			curves[ip] = attackCurves[ip];
			curves[ip][curves[ip].size()-1] = idealPositions[ip][idealPositions[ip].size()-1];
		}

	} 
	else 
	{
		for (int ip = 0; ip < inputs.size(); ++ip) 
			curves[ip] = idealPositions[ip];
	}

	updateFlag = true;
	if (hasGravity) g = data->gravity;
	else g = 0;
	hasRigid = data->rigidness;

	for (int ip = 0; ip < inputs.size(); ++ip) 
	{

		for (int i = 0; i < chainSize; ++i) 
		{
			idealPositions[ip][i] = inputs[ip]->positions[i + index1];
			lastPositions[ip][i] = currentPositions[ip][i];
		}

		for (int i = 0; i < inputs[ip]->positions.size(); ++i)
			outputs[ip]->positions[i] = inputs[ip]->positions[i];
	}

	int numReps = 10;
	ccc = 0;
	
	for (int k = 0; k < numReps-1; ++k) 
	{
		double timeInc = ((double) k / numReps) * (1 / fps);
		
		if (!lookSolver)	
			solve2(time + timeInc); 
		else				
			solve3(time + timeInc);

		if (k == numReps-3) 
			ccc=1;
		else 
			ccc = 0;

	}

	/*for (int ip = 0; ip < inputs.size(); ++ip) {
		for (int i = 0; i < chainSize; ++i) {
			if (nextVerlet != NULL) nextVerlet->currentPositions[ip][i] = currentPositions[ip][i];
			if (nextVerlet != NULL) nextVerlet->lastPositions[ip][i] = lastPositions[ip][i];
		}
	}*/
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
								addSpringBetweenTwoJoints(ip,sk2,i,j,radius*2,2,deltaTime,2,2,1);
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
		int relaxSteps = 20;
		int neighbourDistance = 2;
		int bodyRigidness = 2;
		int neckRigidness = 4;
		if (moveFlag) scaleCurve();

		// Solve all inputs
		for (int ip = 0; ip < inputs.size(); ++ip) {
			Chain* c = inputs[ip];

			if (updateFlag) {
				int headIndex = currentPositions[ip].size()-1;
				int lastNode = attackCurves[ip].size()-1;
				double distBetweenLastNodes = (curves[ip][lastNode] - curves[ip][lastNode-1]).norm();
				double distBetweenLastJoints = (restPositions[ip][headIndex] - restPositions[ip][headIndex-1]).norm();
				if ( distBetweenLastNodes > distBetweenLastJoints && attackFlag)
					placeAttackNode(ip);

				double maxDist = (restPositions[ip][headIndex] - restPositions[ip][5*restPositions[ip].size()/10]).norm();
				double minDist = (restPositions[ip][headIndex] - restPositions[ip][8*restPositions[ip].size()/10]).norm();

				int n = positioningStrengths.size();
				int n1 = n/3;
				int n2 = n-3;

				// Degradado pesos proyecciones
				for (int ii = 0; ii < positioningStrengths.size(); ++ii) {
					if (ii >= headIndex - 5)
						positioningStrengths[ii] = 0;
					else positioningStrengths[ii] = (double)(ii) / 20;
					/*if (ii >= n2) positioningStrengths[ii] = 1;
					else if (ii < n1) positioningStrengths[ii] = 0;
					else positioningStrengths[ii] = (double)(ii - n1) / (n2 - n1);*/
				}
			}

			int mi;
			for (int k = 0; k < relaxSteps; ++k) {

				int hi = currentPositions[ip].size()-1;
				currentPositions[ip][hi] = idealPositions[ip][hi];
				currentPositions[ip][hi-1] = idealPositions[ip][hi-1];
				currentPositions[ip][0] = idealPositions[ip][0];
				currentPositions[ip][1] = idealPositions[ip][1];

				// Rigidness for the "rope behaviour"

				// Add springs
				for (int i = hi-2; i >= hi-5; --i) {
					double dist = (restPositions[ip][hi] - restPositions[ip][i]).norm();
					Vector3d desiredPoint = idealPositions[ip][hi] + (idealPositions[ip][hi-1] - idealPositions[ip][hi]).normalized() * dist;
					
					addSpringToPoint (ip, i, 1, desiredPoint, 0, deltaTime, 0, 0.5);
				}

				for (int i = hi; i >= 0; --i) {
					int step = (i >= hi-1)? neckRigidness : bodyRigidness;
					//if (i == hi) step = 10;
					for (int j = i - step; j <= i+step; ++j) {
						if (j < 0 || j > hi || j == i) continue;
						int minI = 2, minJ = 2;
						double mult = 1;
						if (i >= hi-neckRigidness+1) {
							minI = 30;
							//if (i == hi) mult = 1.3 - (0.1 * abs(hi - j));
						}
						if (i == hi) mult = 4;
						if (j >= hi-1) minJ = 30;
						double desiredDist1 = (restPositions[ip][i] - restPositions[ip][j]).norm();
						addSpringBetweenTwoJoints(ip,ip,i,j,desiredDist1,0,deltaTime,minI,minJ,mult);			
					}
				}

				/*for (int i = 0; i < hi; ++i) {
					int step = (i == hi)? neckRigidness : bodyRigidness;
					for (int j = i - step; j <= i + step; ++j) {
						int minI = 1, minJ = 1;
						if (i != j && j >= 0 && j <= hi) {
							if (i >= hi-2) minI = 30;
							if (j >= hi-2) minJ = 30;
							double desiredDist1 = (restPositions[ip][i] - restPositions[ip][j]).norm();
							addSpringBetweenTwoJoints(ip,ip,i,j,desiredDist1,0,deltaTime,minI,minJ,1);
						}
					}
				}*/

				/*for (int i = 0; i <= hi; ++i) {
					int step = bodyRigidness;
					if (i >= currentPositions[ip].size()-2) step = neckRigidness;
					for (int j = i-step; j <= i+step; ++j) {
						if (i >= currentPositions[ip].size()-2 && j >= currentPositions[ip].size()-2) continue;

						if (i != j && j >= 0 && j <= hi) {
							double mult = 1;
							int minJ = 1;
							int minI = 1;
							if (i >= hi-1) minI = 30;
							if (j >= hi-1) minJ = 30;
							double desiredDist1 = (restPositions[ip][i] - restPositions[ip][j]).norm();
							addSpringBetweenTwoJoints(ip,ip,i,j,desiredDist1,0,deltaTime,minI,minJ,mult);
						}

					}
				}*/

				// Serpenteo
				for (int i = 0; i < hi; ++i) {
					double minDist = -1;
					int minIndex = 0;
					Vector3d p = currentPositions[ip][i];
					Vector3d proj;


					for (int j = lastMinIndexes[i]-1; j <= lastMinIndexes[i]+1; ++j) {
					//for (int j = 0; j < idealPositions[ip].size(); ++j) {
						if (j < 0 || j >= curves[ip].size()-1) continue;
						Vector3d v1 = curves[ip][j];
						Vector3d v2 = curves[ip][j+1];
						int dotProd1 = (v2-v1).dot(p-v1);
						int dotProd2 = (v1-v2).dot(p-v2);
						 
						if ((dotProd1 >= 0 && dotProd2 >= 0) || 
							(dotProd1 >= 0 && dotProd2 < 0 && j >= curves[ip].size()-2)) {

							Vector3d n = (v2 - v1).normalized();
							Vector3d projection = ((v1-p) - ((v1-p).dot(n))*n);
							Vector3d pf = p + projection;
							if ((minDist == -1 || projection.norm() < minDist)) {
								minDist = projection.norm();
								proj = pf;
								minIndex = j;
								mi = minIndex;
							}
						}
					}


					//for (int j = 0; j < idealPositions[ip].size(); ++j) {
					for (int j = lastMinIndexes[i]-1; j <= lastMinIndexes[i]+1; ++j) {
						if (j < 0 || j > curves[ip].size()-1) continue;
						if (minDist == -1 || (p - curves[ip][j]).norm() < minDist) {
							proj = curves[ip][j];
							minDist = (p-curves[ip][j]).norm();
							minIndex = j;
							mi = minIndex;
						}
					}

					if (k == relaxSteps-1) {
						lastMinIndexes[i] = mi;
					}

					//if (minIndex >= idealPositions[ip].size()-3) desvinculado = min(desvinculado,i);

					if (k == relaxSteps-1 && ccc == 1 && i == 0) {
						/*GLUquadricObj *quadric;
						quadric = gluNewQuadric();
						gluQuadricDrawStyle(quadric, GLU_LINE );
						glDisable(GL_LIGHTING);
						glColor3f(0,1,0);
						for (int j = 0; j < curves[ip].size(); ++j) {
							glPushMatrix();
							Vector3d pos = curves[ip][j];
							glTranslated(pos.x(), pos.y(), pos.z());
							double radius = 10;
							gluSphere(quadric,radius,8,8);
							glPopMatrix();
						}
						glEnable(GL_LIGHTING);*/
					}

					if (k == relaxSteps-1 && ccc == 1 && i < desvinculado && positioningStrengths[i] > 0) {

						/*GLUquadricObj *quadric;
						quadric = gluNewQuadric();
						gluQuadricDrawStyle(quadric, GLU_LINE );
						glDisable(GL_LIGHTING);
						glColor3f(1,1,0);
						glPushMatrix();
						glTranslated(proj.x(), proj.y(), proj.z());
						double radius = 8;
						gluSphere(quadric,radius*positioningStrengths[i],8,8);
						glPopMatrix();

						glEnable(GL_LIGHTING);*/
					}



					//if (i < desvinculado) positioningStrengths[i]*2
					addSpringToPoint(ip,i,0,proj, 1, deltaTime, 2, positioningStrengths[i]);
				}

				// Collisions
				for (int i = 0; i < hi; ++i) {
					for (int sk2 = 0; sk2 < currentPositions.size(); ++sk2) {
						for (int j = 0; j < currentPositions[sk2].size(); ++j) {
							if (ip == sk2 && abs(i-j) > neighbourDistance && i < 15) {
								float radius = 12;
								addSpringBetweenTwoJoints(ip,sk2,i,j,radius*2,2,deltaTime,2,2,1);
							}
						}
					}
				}

				updateFlag = false;
			}	// end of relaxing loop

			ccc = 1;
			// Update points
			for (int i = 0; i < currentPositions[ip].size(); ++i) {
				Vector3d velocity = (currentPositions[ip][i] - lastPositions[ip][i]);
				velocity *= velocityDamping;
				Vector3d acceleration = Vector3d(0,g,0);
				acceleration = Vector3d(0,0,0);
				//if (i == 0) velocity = Vector3d(0,0,0);
				if (velocity.norm() < 0.0005) velocity = Vector3d(0,0,0);
				velocity = Vector3d(0,0,0);
				Vector3d nextPos = currentPositions[ip][i] + velocity + acceleration * timeSquared * 0.5;
				lastPositions[ip][i] = currentPositions[ip][i];
				currentPositions[ip][i] = nextPos;
			}

		}	// end of input loop

		for (int ip = 0; ip < inputs.size(); ++ip) {
			for (int i = 0; i < chainSize; ++i)
				outputs[ip]->positions[i + index1] = currentPositions[ip][i];
		}
	}


void SolverVerlet::addSkeleton(skeleton* s, Chain* c)
{
	chainSize = index2 - index1;
	int nextSK = restPositions.size();

	restPositions.push_back(vector<Vector3d>(chainSize));
	lastPositions.push_back(vector<Vector3d>(chainSize));
	currentPositions.push_back(vector<Vector3d>(chainSize));
	idealPositions.push_back(vector<Vector3d>(chainSize));
	curves.push_back(vector<Vector3d>(chainSize));
	attackCurves.push_back(vector<Vector3d>(chainSize));
		
	positioningStrengths = rigidnessStrengths = vector<double> (chainSize, 0);
	lastMinIndexes = vector<int> (chainSize, 0);

	for (int i = 0; i < chainSize; ++i) 
	{
		currentPositions[nextSK][i] = lastPositions[nextSK][i] = restPositions[nextSK][i] = c->positions[i];
		lastMinIndexes[i] = i;
	}
}

void SolverVerlet::setPositions(skeleton* s)
{
	chainSize = index2 - index1;

	restPositions.resize(inputs.size());
	lastPositions.resize(inputs.size());
	currentPositions.resize(inputs.size());
	idealPositions.resize(inputs.size());
	curves.resize(inputs.size());
	attackCurves.resize(inputs.size());

	positioningStrengths = vector<double> (chainSize, 1);

	for (int ip = 0; ip < inputs.size(); ++ip) 
	{
		restPositions[ip].resize(chainSize);
		lastPositions[ip].resize(chainSize);
		currentPositions[ip].resize(chainSize);
		idealPositions[ip].resize(chainSize);

		for (int i = 0; i < chainSize; ++i) 
		{
			currentPositions[0][i] = lastPositions[0][i] = restPositions[0][i] = s->joints[i]->translation;
			positioningStrengths[i] = (1 - 0.03*i);
		}
	}
}

void SolverVerlet::solve(SolverData* data) 
{
	if (hasGravity) 
		g = data->gravity;
	else 
		g = 0;
		
	time = data->time;
	fps = data->fps;
	solve();
}

void SolverVerlet::updateDirtyness() 
{
	dirtyFlag = true;
	propagateDirtyness();
}