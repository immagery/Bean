#pragma once
#include "Solver.h"

class SolverSinusoidal : public Solver {
public:
	// Wave parameters
	double _amplitude, _freq, phase, longitude;
	// Axis of the oxilation
	int dimension;		// 0=X, 1=Y, 2=Z
	Vector3d axis;		// relative to vector defined by lastPos - initPos
	// Wave modulation
	int thresh1, thresh2, thresh3;
	double multAmp, multFreq;

	// Constructors and destructors
	SolverSinusoidal(void) : Solver(), multAmp(1), multFreq(1) {}
	SolverSinusoidal(double a, double f, double ph) : Solver(), _amplitude(a), _freq(f), phase(ph), dimension(0), multAmp(1), multFreq(1) {}
	~SolverSinusoidal(void) {}

	// Solving
	void solve(SolverData* data) {
		time = data->time;
		solve();
	}

	void updateDirtyness() {
		dirtyFlag = true;
		propagateDirtyness();
	}

	virtual void solve() {
		double amplitude = _amplitude * multAmp;
		double freq = _freq * multFreq;

		for (int i = 0; i < inputs.size(); ++i) {
			Chain* ichain = inputs[i];
			for (int j = 0; j < ichain->positions.size(); ++j) outputs[i]->positions[j] = ichain->positions[j];
			for (int j = index1; j <= index2; ++j) {
				double mAmp;

				if (j < thresh1) mAmp = 0;
				else if (j < thresh2) mAmp = amplitude * (double)(j - thresh1) / (thresh2 - thresh1);
				else mAmp = amplitude;

				//else if (j < thresh3) mAmp = amplitude;
				//else if (thresh3 != 0) mAmp = 0;

				double inc = (mAmp * sin(freq*time - j/longitude));
				Vector3d increment;
				if (dimension == 0)			increment = Vector3d(inc,0,0);
				else if (dimension == 1)	increment = Vector3d(0,inc,0);
				else if (dimension == 2)	increment = Vector3d(0,0,inc);
				outputs[i]->positions[j] = ichain->positions[j] + increment;
			}
		}
	}
};

