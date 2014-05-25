#ifndef SNAKE_H
#define SNAKE_H

#include "snakeVertebra.h"
#include <DataStructures/ParticlePool.h>
#include <DataStructures/Modelo.h>
#include <DataStructures/skeleton.h>
#include <snakeBrain.h>

class snake 
{

public:
	snake();
	~snake();

	void initGeneric(Vector3d initPt);
	void initFromSkeleton(Modelo* m, skeleton* skt);

	void move();
	void think();
	void buildSkeleton();

	void moveToPosition(Vector3d pos);
	void changeOrientation(Vector3d or);

	// Render functions
    void drawFunc();
	void setColor(float r, float g, float b) {color[0] = r; color[1] = g; color[2] = b;}

    // Pool of particles for snake simulation
	ParticlePool particles;
	
	// vertebra structure for better control the snake
	vector<snakeVertebra> vertebras;

	// References for particle <-> joint atachment
	map<int, int> sktRef;

	// snake brain
	snakeBrain brain;

	//Reference to the model
	Modelo* m_model;
	skeleton* m_skt;

	map<int, int> joint2Vert;

    // Atributes
    axis orientation;

    // rendering info
    renderMode mode;
	Vector3f color;

};

#endif // SNAKE_H