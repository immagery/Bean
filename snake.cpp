#include "snake.h"

#include <PPreConstraintVertebrae.h>

// Move all the snake, for initization porpouses
void snake::moveToPosition(Vector3d pos)
{
	Vector3d posRel = brain.headVertebra->center->position;

	for(int i = 0; i< particles.pool.size(); i++)
	{
		particles.pool[i].position = (particles.pool[i].position-posRel) + pos;
	}

	brain.baseAxis.pos = brain.baseAxis.pos-posRel + pos;
	brain.headAxis.pos = brain.headAxis.pos-posRel + pos;

	brain.objective = brain.headAxis.pos;
}

void snake::changeOrientation(Vector3d or)
{
	Vector3d dir(1,0,0);
	Quaterniond q = Quaterniond::Identity();

	if(or.norm() > 0) 
		q.setFromTwoVectors(dir, or.normalized());
	
	brain.headAxis.rot = q;	
}

void snake::initFromSkeleton(Modelo* m, skeleton* skt)
{

	if(!skt || ! skt->joints.size()> 0) return;

	m_model = m;
	m_skt = skt;
	
	// Cogemos algunas referencias a saco.
	skt->root->computeWorldPos();
	int numBones = 23;

	Vector3d upDirection(0,1,0);
	Vector3d headDirection = (skt->joints[22]->worldPosition-skt->joints[21]->worldPosition).normalized();
	Vector3d ribDirection = headDirection.cross(upDirection);

	float ribWidth = 2;
	float bodyHeight = 1.3;

	// Init snake vertebras and so on
	int numAssociatedParticles = 5; 
	//particles.resizePool(numVertebras*numAssociatedParticles);
	//vertebras.resize(numVertebras);

	particles.resizePool(numBones*numAssociatedParticles*100);

	int id = 0;
	for(int i = 0; i< numBones; i++)
	{
		/*
		Vector3d pt00 = initPt + Vector3d(40*i, 50, 0);
		Vector3d pt01 = pt00 + Vector3d(-20,  0,  0);
		Vector3d pt02 = pt00 + Vector3d(  0, 15,  0);
		Vector3d pt03 = pt00 + Vector3d(  0,  0, 35);
		Vector3d pt04 = pt00 + Vector3d(  0,  0,-35);
		*/

		int jointId = numBones-i-1;

		Vector3d pt00 = skt->joints[jointId]->worldPosition;
		Vector3d pt01 = pt00 + Vector3d(1,  0,  0) * bodyHeight;
		Vector3d pt02 = pt00 + Vector3d(  0, 1,  0)* bodyHeight/0.5;
		Vector3d pt03 = pt00 + Vector3d(  0,  0, 1) * ribWidth;
		Vector3d pt04 = pt00 + Vector3d(  0,  0,-1) * ribWidth;

		vertebras.resize(vertebras.size()+1);
		vertebras.back().init(pt00, pt01, pt02, pt03, pt04, particles);

		joint2Vert[jointId] = vertebras.size()-1;

		
		// Rellenamos con las vertebras que sean necesarias.
		if(i < numBones - 1)
		{
			int nextJointId = numBones-i-2;
			Vector3d pt10 = skt->joints[nextJointId]->worldPosition;

			Vector3d bone = pt10 - pt00;
			float numElements = floor(bone.norm()/(ribWidth/0.5));
			float length = bone.norm()/numElements;

			Vector3d boneDir = bone.normalized();

			for(int h = 1; h < numElements; h++)
			{
				Vector3d newpt00 = pt00 + boneDir*h;
				pt01 = pt00 + Vector3d(1,  0,  0) * bodyHeight * 3;
				pt02 = pt00 + Vector3d(  0, 1,  0)* bodyHeight*10;
				pt03 = pt00 + Vector3d(  0,  0, 1) * ribWidth;
				pt04 = pt00 + Vector3d(  0,  0,-1) * ribWidth;

				vertebras.resize(vertebras.size()+1);
				vertebras.back().init(pt00, pt01, pt02, pt03, pt04, particles);
			}

			printf("Num of subdivisions: %d\n", numElements); 
		}
	}


	snakeVertebra* head =  &vertebras[0];
	snakeVertebra* base =  &vertebras.back();

	brain.setHeadVertebra(head);
	brain.initTime();

	brain.setBaseVertebra(base);

	particles.worldPos = base->center->position - Vector3d(0,bodyHeight,0);
	particles.worldY = (base->y->position - base->center->position).normalized();
	particles.worldPos -= particles.worldY*bodyHeight;

	for(int i = 0; i < numBones; i++)
	{
		snakeVertebra& v1 = vertebras[i];

		// Restricciones globales de cada vertebra
		particles.modelConstraints.push_back(new pConstraintAngle(v1.center, v1.y, base->center, base->y, 0));

		// Restricciones en
		if( i < vertebras.size()-1)
		{
			snakeVertebra& v2 = vertebras[i+1];
			// Distancia entre vertebras
			particles.modelConstraints.push_back(new pConstraintDistance(v1.center, v2.center));

			// Alineacion de eje x,y entre vertebras consecutivas
			particles.modelConstraints.push_back(new pConstraintAngle(v1.center, v1.x, v2.center, v2.x, 0));
			
			if(i+2 < vertebras.size()-1)
			{
				snakeVertebra& v3 = vertebras[i+2];
				particles.modelConstraints.push_back(new pConstraintAngle(v1.center, v2.center, v2.center, v3.center, 0));
				particles.modelConstraints.push_back(new pConstraintAngle(v1.center, v2.center, v1.center, v3.center, 0));
			}
			particles.modelConstraints.push_back(new pConstraintAngle(v1.center, v1.y, v2.center, v2.y, 0));

			
			particles.modelConstraints.push_back(new pConstraintDistance(v1.y, v2.y));

			// Alineacion de direccion x en vertebras consecutivas para dar rigidez
			particles.modelConstraints.push_back(new pConstraintAngle(v1.center, v1.x, v1.center, v2.center, 180));


			// Preconstraints de selfcolision hacia los hijos (costillas)
			PPreConstraintVertebrae* pc = new PPreConstraintVertebrae(v2.zl, v1.center, v1.x, true);
			particles.preColisionConstraints.push_back((PPreConstraint*)pc);

			PPreConstraintVertebrae* pc2 = new PPreConstraintVertebrae(v2.zr, v1.center, v1.x, true);
			particles.preColisionConstraints.push_back((PPreConstraint*)pc2);

			PPreConstraintVertebrae* pc3 = new PPreConstraintVertebrae(v1.zl, v2.center, v2.x, false);
			particles.preColisionConstraints.push_back((PPreConstraint*)pc3);

			PPreConstraintVertebrae* pc4 = new PPreConstraintVertebrae(v1.zr, v2.center, v2.x, false);
			particles.preColisionConstraints.push_back((PPreConstraint*)pc4);

		}
	}

	printf("[CONSTRUCCION SERPIENTE] - restriccciones: %d\n", particles.modelConstraints.size());
}


void snake::initGeneric(Vector3d initPt)
{
	// Init snake vertebras and so on
	int numVertebras = 17;
	int numAssociatedParticles = 5; 
	particles.resizePool(numVertebras*numAssociatedParticles);

	vertebras.resize(numVertebras);

	int id = 0;
	for(int i = 0; i< numVertebras; i++)
	{
		Vector3d pt00 = initPt + Vector3d(40*i, 50, 0);
		Vector3d pt01 = pt00 + Vector3d(-20,  0,  0);
		Vector3d pt02 = pt00 + Vector3d(  0, 15,  0);
		Vector3d pt03 = pt00 + Vector3d(  0,  0, 35);
		Vector3d pt04 = pt00 + Vector3d(  0,  0,-35);

		vertebras[i].init(pt00, pt01, pt02, pt03, pt04, particles);
	}

	brain.setHeadVertebra(&vertebras[0]);
	brain.initTime();
	snakeVertebra* head =  &vertebras[0];

	for(int i = 0; i < numVertebras; i++)
	{
		snakeVertebra& v1 = vertebras[i];

		// Restricciones globales de cada vertebra
		particles.modelConstraints.push_back(new pConstraintAngle(v1.center, v1.y, head->center, head->y, 0));

		// Restricciones en
		if( i < numVertebras-1)
		{
			snakeVertebra& v2 = vertebras[i+1];
			// Distancia entre vertebras
			particles.modelConstraints.push_back(new pConstraintDistance(v1.center, v2.center));

			// Alineacion de eje x,y entre vertebras consecutivas
			particles.modelConstraints.push_back(new pConstraintAngle(v1.center, v1.x, v2.center, v2.x, 0));
			particles.modelConstraints.push_back(new pConstraintAngle(v1.center, v1.y, v2.center, v2.y, 0));

			// Alineacion de direccion x en vertebras consecutivas para dar rigidez
			particles.modelConstraints.push_back(new pConstraintAngle(v1.center, v1.x, v1.center, v2.center, 180));

			// Preconstraints de selfcolision hacia los hijos (costillas)
			/*
			PPreConstraintVertebrae* pc = new PPreConstraintVertebrae(v2.zl, v1.center, v1.x);
			particles.preColisionConstraints.push_back((PPreConstraint*)pc);

			PPreConstraintVertebrae* pc2 = new PPreConstraintVertebrae(v2.zr, v1.center, v1.x);
			particles.preColisionConstraints.push_back((PPreConstraint*)pc2);
			*/

		}
	}
}

snake::snake()
{
	// Init Data
	vertebras.clear();
}

snake::~snake(void)
{
}


void snake::drawFunc() 
{
	particles.drawFunc();

	brain.drawFunc();

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

void snake::think()
{
	
}

void snake::move()
{
	// Launch IA step
	brain.think(particles.timeStep);

	// Launch simulation step
	particles.simulate(particles.timeStep);	

	// Build skeleton
	buildSkeleton();
}

void snake::buildSkeleton()
{
	int vertebrasCount = vertebras.size();
	
	Vector3d lastPosition;
	Vector3d lastIncrement;

	Quaterniond rotComplete = Quaterniond::Identity();
	Vector3d translationComplete = Vector3d(0,0,0);

	map<int, int>::iterator it = joint2Vert.begin();
	map<int, int>::iterator it2 = joint2Vert.begin();

	for(; it != joint2Vert.end(); it++)
	{
		it2 = it;
		it2++;

		joint* jt = m_skt->joints[it->first];

		int vertId = it->second;

		snakeVertebra& vert = vertebras[vertId];

		Vector3d y = Vector3d(0,1,0);
		Vector3d x = Vector3d(1,0,0);

		Vector3d xRef;
		// Taking the axis as a reference... maybe is not the best solution, 
		// because is not allways well aligned
		if( vertId == 0 || vertId == vertebras.size()-1 )//it == joint2Vert.begin() || it2 == joint2Vert.end())
			xRef = vert.x->position - vert.center->position;
		else
		{
			map<int, int>::iterator it3 = it;
			it3--;
			//snakeVertebra& vert2 = vertebras[it2->second];
			//snakeVertebra& vert3 = vertebras[it3->second];
			snakeVertebra& vert2 = vertebras[vertId-1];
			snakeVertebra& vert3 = vertebras[vertId+1];

			Vector3d xRef0 =  (vert.center->position - vert2.center->position).normalized();
			Vector3d xRef1 =  (vert3.center->position - vert.center->position).normalized();

			xRef = -(xRef0 + xRef1)/2;
		}
		
		Vector3d yRef = vert.y->position - vert.center->position;

		xRef.normalize();
		yRef.normalize();

		Quaterniond q1 = Quaterniond::Identity();
		q1.setFromTwoVectors(x, xRef);
			
		y = q1._transformVector(y);
		Quaterniond q2 = Quaterniond::Identity();
		q2.setFromTwoVectors(y, yRef);

		m_skt->joints[it->first]->qrot = rotComplete.inverse()*q1*q2;
		m_skt->joints[it->first]->pos = rotComplete.inverse()._transformVector(vert.center->position-translationComplete);	
		translationComplete = vert.center->position;

		rotComplete = q1*q2;
	} 

	m_skt->root->computeWorldPos();
}
