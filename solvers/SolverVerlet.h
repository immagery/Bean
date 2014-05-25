#pragma once
#include "Solver.h"
#include "skeleton.h"

// todo: poner un flag para poder hacer cálculos sin sobreescribir durante un frame
// usar para no cambiar el segmento entre pasos de relajación

class SolverVerlet : public Solver 
{
public:
	vector<vector<Vector3d> > restPositions;		// resting positions to compute the desired rest distance between links
	vector<vector<Vector3d> > lastPositions;					// positions in last time
	vector<vector<Vector3d> > currentPositions;
	vector<vector<Vector3d> > idealPositions;

	double lastTime, g, velocityDamping, fps;
	bool hasGravity;
	bool hasRigid;
	bool lookSolver;		// para hacer casos especiales

	int ccc;
	bool printStuff;
	bool updateFlag;
	bool moveFlag;

	// Attack curve
	vector<vector<Vector3d> > attackCurves;
	vector<vector<Vector3d> > curves;

	// Spring parameters: Strength, Damping, Stiffness
	double distS, distD, distStiff;		// springs between nodes
	double posS, posD, posStiff;		// springs between nodes and ideal positions
	double colS, colD, colStiff;		// dynamic springs to avoid collisions
	double rigidS, rigidD, rigidStiff;
	double slider;
	vector<double> positioningStrengths;
	vector<double> rigidnessStrengths;

	int desvinculado;
	int minVinculado;
	vector<int> lastMinIndexes;
	bool attackFlag;
	Vector3d lookPoint;
	bool moving;
	
	SolverVerlet* nextVerlet;		// points to next verlet in order to send current and last positions

	SolverVerlet(int _id);
	~SolverVerlet();

	
	void addSpringBetweenTwoJoints( int sk1, int sk2, int i1, int i2, 
									double desiredDist, int springType, 
									double deltaTime, int min1, int min2, 
									double multiplier);

	void addSpringBetweenTwoJoints3D(int sk1, int sk2, int i1, int i2, 
									 double desiredDist, int springType, 
									 double deltaTime, int min1, int min2, 
									 double multiplier);

	void addSpringToPoint ( int sk1, int i1, double desiredDist, 
							Vector3d p, int springType, double deltaTime, 
							int min1, double multiplier);

	virtual void solve();
	void solve2(double ttime);
	void solve3(double ttime);

	void buildAttackCurves();
	void placeAttackNode(int sk);
	void disableAttack();
	void scaleCurve();
	
	vector< pair<int,Vector3d> > SolverVerlet::solveVerlet(double time, vector<SolverVerlet*>& verlets, int skID);
	void solveVerlet2 (double time, SolverData* data);
	
	virtual void solve(double time) {}

	void addSkeleton(skeleton* s, Chain* c);
	void setPositions(skeleton* s);

	void solve(SolverData* data);

	void updateDirtyness();

};

