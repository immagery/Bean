#ifndef VERTEBRA_H
#define VERTEBRA_H

#include "DataStructures/ParticlePool.h"
#include "DataStructures\PConstraintAngle.h"
#include "DataStructures\PConstraintDistance.h"

class snakeVertebra {

public:
    snakeVertebra();
	~snakeVertebra();

	// Render functions
    void drawFunc();
	void setColor(float r, float g, float b) {color[0] = r; color[1] = g; color[2] = b;}

	void init(Vector3d pt00, Vector3d pt01, Vector3d pt02,
			  Vector3d pt03, Vector3d pt04, ParticlePool& particles);

	void setAsHead();

    // Basic atributes.
	Particle* center;
	Particle* x;
	Particle* y;
	Particle* y_prima;
	Particle *zl, *zr;

    // Atributes
    axis orientation;

    // rendering info
    renderMode mode;
	Vector3f color;

};

#endif // VERTEBRA_H