#pragma once
#include "BaseSystem.h"
#include <util/util.h>

class Particle
{
public:
	Particle();
	Particle(double x, double y, double z, double velX, double velY, double velZ, double mass);
	void setPos(double x, double y, double z);
	void getPos(double* p);
	void setVel(double x, double y, double z);
	void getVel(double* p);
	void setMass(double m);
	double getMass();
	Vector PrevPosition;
	Vector PrevVelocity;
	double ColorR;
	double ColorG;
	double ColorB;
	double alpha;
	double particleLifeTime;
	double timeBeenAlive;

private:
	Vector m_Position;
	Vector m_Velocity;
	double m_Mass;

};

