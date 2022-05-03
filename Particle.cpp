#include "Particle.h"

Particle::Particle()
{
	setVector(m_Position, 0.0, 0.0, 0.0);
	setVector(m_Velocity, 0.0, 0.0, 0.0);
	m_Mass = 1.0;
	timeBeenAlive = 0.0;
}

Particle::Particle(double x, double y, double z, double velX, double velY, double velZ, double mass)
{
	setVector(m_Position, x, y, z);
	setVector(m_Velocity, velX, velY, velZ);
	m_Mass = mass;
}

void Particle::setPos(double x, double y, double z)
{
	setVector(m_Position, x, y, z);
}

void Particle::getPos(double* p)
{
	VecCopy(p, m_Position);
}

void Particle::setVel(double x, double y, double z)
{
	setVector(m_Velocity, x, y, z);
}

void Particle::getVel(double* p)
{
	VecCopy(p, m_Velocity);
}

void Particle::setMass(double m)
{
	m_Mass = m;
}

double Particle::getMass()
{
	return m_Mass;
}
