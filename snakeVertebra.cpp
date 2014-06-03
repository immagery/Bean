#include "snakeVertebra.h"
#include "ParticlePool.h"
#include "PConstraint.h"
#include "PConstraintAngle.h"
#include "PConstraintDistance.h"

snakeVertebra::snakeVertebra()
{
	center = NULL;
	x = NULL;
	y = NULL;
	zl = NULL;
	zr = NULL;
}

snakeVertebra::~snakeVertebra(void)
{
}


void snakeVertebra::drawFunc() 
{
	/*
	glColor3f(color.x(), color.y(), color.z());
    if(mode == RENDER_POINT)
    {
        glPointSize(8);
	    glBegin(GL_POINTS);
		    glVertex3d(position.x(), position.y(), position.z());
	    glEnd();
    }
	*/
}

void snakeVertebra::initAsHead(Vector3d pt00, Vector3d pt01, Vector3d pt02,
								Vector3d pt03, Vector3d pt04, ParticlePool& particles)
{
	//// Creación de particulas

	// Centro
	int id = particles.addParticle(pt00);
	center = &particles.pool[id];

	// Direccion
	id = particles.addParticle(pt01);
	particles.pool[id].setColor(1,0,0);
	particles.pool[id].onlyRestrictionForces = 1.0;
	x = &particles.pool[id];
	

	id = particles.addParticle(pt02);
	particles.pool[id].setColor(0,1,0);
	particles.pool[id].onlyRestrictionForces = 1.0;
	y = &particles.pool[id];

	// Costillas
	id = particles.addParticle(pt03);
	particles.pool[id].setColor(0,0,1);
	particles.pool[id].onlyRestrictionForces = 1.0;
	zl = &particles.pool[id];

	id = particles.addParticle(pt04);
	particles.pool[id].setColor(0,0,1);
	particles.pool[id].onlyRestrictionForces = 1.0;
	zr = &particles.pool[id];


	//// Restricciones dentro de la articulacion
	particles.modelConstraints.push_back(new pConstraintDistance(center, x));
	particles.modelConstraints.push_back(new pConstraintDistance(center, y));
	particles.modelConstraints.push_back(new pConstraintDistance(center, zl));
	particles.modelConstraints.push_back(new pConstraintDistance(center, zr));
	particles.modelConstraints.push_back(new pConstraintDistance(zl, zr));

	//particles.modelConstraints.push_back(new pConstraintAngle(center, x, center, y, 90));
	particles.modelConstraints.push_back(new pConstraintAngle(center, x, center, zl, 90));
	particles.modelConstraints.push_back(new pConstraintAngle(center, x, center, zr, 90));

	particles.modelConstraints.push_back(new pConstraintAngle(center, y, center, zl, 120));
	particles.modelConstraints.push_back(new pConstraintAngle(center, y, center, zr, 120));
	particles.modelConstraints.push_back(new pConstraintAngle(center, zr, center, zl, 120));
	
}

void snakeVertebra::init(Vector3d pt00, Vector3d pt01, Vector3d pt02,
						 Vector3d pt03, Vector3d pt04, ParticlePool& particles)
{
	//// Creación de particulas

	// Centro
	int id = particles.addParticle(pt00);
	center = &particles.pool[id];

	// Direccion
	id = particles.addParticle(pt01);
	particles.pool[id].setColor(1,0,0);
	particles.pool[id].onlyRestrictionForces = 1.0;
	x = &particles.pool[id];
	

	id = particles.addParticle(pt02);
	particles.pool[id].setColor(0,1,0);
	particles.pool[id].onlyRestrictionForces = 1.0;
	y = &particles.pool[id];

	// Costillas
	id = particles.addParticle(pt03);
	particles.pool[id].setColor(0,0,1);
	particles.pool[id].onlyRestrictionForces = 1.0;
	zl = &particles.pool[id];

	id = particles.addParticle(pt04);
	particles.pool[id].setColor(0,0,1);
	particles.pool[id].onlyRestrictionForces = 1.0;
	zr = &particles.pool[id];

	//// Restricciones dentro de la articulacion
	particles.modelConstraints.push_back(new pConstraintDistance(center, x));
	particles.modelConstraints.push_back(new pConstraintDistance(center, y));
	particles.modelConstraints.push_back(new pConstraintDistance(center, zl));
	particles.modelConstraints.push_back(new pConstraintDistance(center, zr));
	//particles.modelConstraints.push_back(new pConstraintDistance(zl, zr));

	particles.modelConstraints.push_back(new pConstraintAngle(center, x, center, y, 90));
	particles.modelConstraints.push_back(new pConstraintAngle(center, x, center, zl, 90));
	particles.modelConstraints.push_back(new pConstraintAngle(center, x, center, zr, 90));

	particles.modelConstraints.push_back(new pConstraintAngle(center, y, center, zl, 120));
	particles.modelConstraints.push_back(new pConstraintAngle(center, y, center, zr, 120));
	particles.modelConstraints.push_back(new pConstraintAngle(center, zr, center, zl, 120));
}

void snakeVertebra::setAsHead()
{
	center->weight = 0;
	x->weight = 0;
	y->weight = 0;
}	