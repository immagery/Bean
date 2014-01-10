#include "SolverHead.h"

void SolverHead::solve(SolverData* data) {
	time = data->time;
	lookPoint = data->lookPoint;
	alpha = data->alpha;
	solve();
}

void SolverHead::updateDirtyness() {
	dirtyFlag = true;
	dirtyFlag = (alpha != data->alpha || lookPoint != data->lookPoint);
	propagateDirtyness();
}

void SolverHead::solve() {
	lookPoint = data->lookPoint;
	alpha = data->alpha;

	for (int i = 0; i < inputs.size(); ++i) {
		Chain* ichain = inputs[i];
		for (int j = 0; j < ichain->positions.size(); ++j)
			outputs[i]->positions[j] = ichain->positions[j];

		Vector3d secondNode = outputs[i]->positions[index2];

		Vector3d lookDirection = lookPoint - secondNode;
		Vector3d pointToMove = secondNode + alpha * lookDirection;

		outputs[i]->positions[index2] = pointToMove;
	}
}