#include "SolverPos.h"


void SolverPos::solve(SolverData* data) {
	
}

void SolverPos:: solve () {
	baseRotation = data->baseRotation;
	baseTranslation = data->baseTranslation;

	for (int ip = 0; ip < inputs.size(); ++ip) {
		// Move base to its world position
		//outputs[ip]->positions[0] = inputs[ip]->positions[0];
		outputs[ip]->positions[0] = restTrans + baseTranslation;

		// Move each joint to its rotated position
		for (int i = 1; i < outputs[ip]->positions.size(); ++i) {
			Vector3d t = (inputs[ip]->positions[i] - inputs[ip]->positions[i-1]);
			outputs[ip]->positions[i] = outputs[ip]->positions[i-1] + baseRotation._transformVector(t);
		}
	}
}