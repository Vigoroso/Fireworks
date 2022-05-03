#include "ParticlePhysicsSimulator.h"
#include <math.h>
#include <cmath>
#include "GlobalResourceManager.h"

ParticlePhysicsSimulator::ParticlePhysicsSimulator(const std::string& name) :
	BaseSimulator(name)
{
}	// ParticlePhysicsSimulator

ParticlePhysicsSimulator::~ParticlePhysicsSimulator()
{
}	// SampleGravitySimulator::~SampleGravitySimulator

int ParticlePhysicsSimulator::step(double time)
{
	Vector* SpringForces = new Vector[m_ParticleSystem->NumSprings];
	Particle* particles = m_ParticleSystem->getParticles();
	bool particlesUpdated = false;
	for (int i = 0; i < m_ParticleSystem->getNumParticles(); i++)
	{
		if (!m_Fixed[i])
		{
			if (particles[i].timeBeenAlive < particles[i].particleLifeTime)
			{
				particlesUpdated = true;
				SetParticleState(i, time);
			}
		}
	}
	delete [] SpringForces;
	mFirstStep = false;

	if (!particlesUpdated)
	{
		MarkedForDelete = true;
	}
	return 0;
}

void ParticlePhysicsSimulator::SetParticleState(int particleIndex, double time)
{

	// update particle lifetime

	Vector totalForce;
	setVector(totalForce, 0.0, 0.0, 0.0);

	Particle* particles = m_ParticleSystem->getParticles();

	// update particle lifetime
	particles[particleIndex].timeBeenAlive += m_TimeStep * 10;

	// calculate forces

	Vector pos;
	particles[particleIndex].getPos(pos);

	Vector fromOrigin;
	VecCopy(fromOrigin, m_ParticleSystem->Origin);
	fromOrigin[1] = fromOrigin[1] - m_ParticleSystem->yRange;
	VecSubtract(fromOrigin, pos, fromOrigin);

	if (mFirstStep)
	{
		// Scale proportionally to distance from explosion (more force to closer particles)

		double a = 0.0;
		double b = 50.0;

		double randomScaled1 = ((2.0 * m_RandForceRange + (double)rand()) / ((double)RAND_MAX / (2.0 * m_RandForceRange)) - m_RandForceRange);
		double randomScaled2 = ((2.0 * m_RandForceRange + (double)rand()) / ((double)RAND_MAX / (2.0 * m_RandForceRange)) - m_RandForceRange);
		double randomScaled3 = ((2.0 * m_RandForceRange + (double)rand()) / ((double)RAND_MAX / (2.0 * m_RandForceRange)) - m_RandForceRange);

		double scale0 = abs(fromOrigin[0] / m_ParticleSystem->xRange);
		double lerp0 = a + scale0 * (b - a);
		lerp0 = lerp0 - (lerp0 * randomScaled1);
		double scale1 = abs(fromOrigin[1] / m_ParticleSystem->yRange);
		double lerp1 = a + scale1 * (b - a);
		lerp1 = lerp1 - (lerp1 * randomScaled2);
		double scale2 = abs(fromOrigin[2] / m_ParticleSystem->zRange);
		double lerp2 = a + scale2 * (b - a);
		lerp2 = lerp2 - (lerp2 * randomScaled3);

		fromOrigin[0] = (fromOrigin[0] > 0) ? lerp0 : -lerp0;
		fromOrigin[1] = (fromOrigin[1] > 0) ? lerp1 : -lerp1;
		fromOrigin[2] = (fromOrigin[2] > 0) ? lerp2 : -lerp2;

		VecAdd(totalForce, totalForce, fromOrigin);
	}

	Vector drag;
	particles[particleIndex].getVel(drag);
	VecScale(drag, m_DragCoef);
	VecAdd(totalForce, totalForce, drag);

	Vector gravity;
	setVector(gravity, 0, m_Gravity, 0);
	VecScale(gravity, particles[particleIndex].getMass());
	VecAdd(totalForce, totalForce, gravity);

	for (int i = 0; i < m_ParticleSystem->NumSprings; i++)
	{
		Spring spring = m_ParticleSystem->Springs[i];

		if (spring.Particle1 == particleIndex)
		{
			Vector totalSpringForce;
			Particle particleI = m_ParticleSystem->getParticles()[spring.Particle1];
			Particle particleJ = m_ParticleSystem->getParticles()[spring.Particle2];
			AddSpringForces(spring, particleI, particleJ, totalSpringForce);
			VecAdd(totalForce, totalForce, totalSpringForce);
		}
		else if (spring.Particle2 == particleIndex)
		{
			Vector totalSpringForce;
			Particle particleI = m_ParticleSystem->getParticles()[spring.Particle2];
			Particle particleJ = m_ParticleSystem->getParticles()[spring.Particle1];
			AddSpringForces(spring, particleI, particleJ, totalSpringForce);
			VecAdd(totalForce, totalForce, totalSpringForce);
		}
	}

	VecScale(totalForce, m_TimeStep / particles[particleIndex].getMass());

	if (m_ActiveIntegrationMethod == symplectic)
	{
		VecCopy(particles[particleIndex].PrevPosition, pos);
		Vector vel;
		particles[particleIndex].getVel(vel);

		Vector newVel;
		VecCopy(newVel, vel);

		VecAdd(newVel, newVel, totalForce);
		particles[particleIndex].setVel(newVel[0], newVel[1], newVel[2]);

		VecScale(newVel, m_TimeStep);
		VecAdd(pos, pos, newVel);
		particles[particleIndex].setPos(pos[0], pos[1], pos[2]);
	}
	else if ((m_ActiveIntegrationMethod == euler) || (time == 0.01))
	{
		VecCopy(particles[particleIndex].PrevPosition, pos);
		Vector vel;
		particles[particleIndex].getVel(vel);

		VecAdd(vel, vel, totalForce);
		particles[particleIndex].setVel(vel[0], vel[1], vel[2]);

		VecScale(vel, m_TimeStep);
		VecAdd(pos, pos, vel);
		particles[particleIndex].setPos(pos[0], pos[1], pos[2]);
	}
	else if (m_ActiveIntegrationMethod == verlet)
	{
		Vector vel;
		particles[particleIndex].getVel(vel);
		VecScale(vel, m_TimeStep);

		Vector newPos;
		VecCopy(newPos, pos);
		VecScale(newPos, 2.0);
		VecSubtract(newPos, newPos, particles[particleIndex].PrevPosition);
		VecScale(totalForce, m_TimeStep);
		VecAdd(newPos, newPos, totalForce);
		particles[particleIndex].setPos(newPos[0], newPos[1], newPos[2]);

		Vector newVel;
		VecCopy(newVel, newPos);
		VecSubtract(newVel, newVel, particles[particleIndex].PrevPosition);
		VecScale(newVel, 1.0 / (2 * m_TimeStep));
		particles[particleIndex].setVel(newVel[0], newVel[1], newVel[2]);

		VecCopy(particles[particleIndex].PrevPosition, pos);
	}
}

void ParticlePhysicsSimulator::Linksystem(std::string name, int numSprings)
{
	BaseSystem* system;
	system = GlobalResourceManager::use()->getSystem(name);
	if (system != nullptr)
	{
		m_ParticleSystem = (ParticleSystem*)system;
		m_ParticleSystem->MaxNumSprings = numSprings;
		m_ParticleSystem->Springs = new Spring[m_ParticleSystem->MaxNumSprings];
		m_Fixed = new bool[m_ParticleSystem->getNumParticles()];
		for (int i = 0; i < m_ParticleSystem->getNumParticles(); i++)
		{
			m_Fixed[i] = false;
		}
	}
}

void ParticlePhysicsSimulator::AddSpringForces(Spring spring, Particle particleI, Particle particleJ, double* totalSpringForce)
{
	Vector posI;
	particleI.getPos(posI);
	Vector posJ;
	particleJ.getPos(posJ);

	Vector sf;
	VecSubtract(sf, posI, posJ);
	double distBetween = VecLength(sf);
	double springCoeff = spring.KS * (spring.RestLength - distBetween);

	VecScale(sf, (1.0 / distBetween));
	VecScale(sf, springCoeff);

	Vector df;
	Vector velI;
	particleI.getVel(velI);
	Vector velJ;
	particleJ.getVel(velJ);
	VecSubtract(df, velI, velJ);

	VecScale(df, spring.KD);

	VecCopy(totalSpringForce, sf);
	VecSubtract(totalSpringForce, totalSpringForce, df);
}

int ParticlePhysicsSimulator::command(int argc, myCONST_SPEC char** argv)
{
	if (argc < 1)
	{
		animTcl::OutputMessage("system %s: wrong number of params.", m_name.c_str());
		return TCL_ERROR;
	}
	else if (strcmp(argv[0], "link") == 0)
	{
		if (argc != 3)
		{
			animTcl::OutputMessage("system %s: wrong number of params for set.", m_name.c_str());
			return TCL_ERROR;
		}
		else
		{
			Linksystem(argv[1], std::stoi(argv[2]));
		}
	}
	else if (strcmp(argv[0], "spring") == 0)
	{
		if (argc != 6)
		{
			animTcl::OutputMessage("system %s: wrong number of params for set.", m_name.c_str());
			return TCL_ERROR;
		}
		else
		{
			m_ParticleSystem->Springs[m_ParticleSystem->NumSprings].Particle1 = std::stoi(argv[1]);
			m_ParticleSystem->Springs[m_ParticleSystem->NumSprings].Particle2 = std::stoi(argv[2]);
			m_ParticleSystem->Springs[m_ParticleSystem->NumSprings].KS = std::stod(argv[3]);
			m_ParticleSystem->Springs[m_ParticleSystem->NumSprings].KD = std::stod(argv[4]);
			m_ParticleSystem->Springs[m_ParticleSystem->NumSprings].RestLength = std::stod(argv[5]);
			if (m_ParticleSystem->Springs[m_ParticleSystem->NumSprings].RestLength < 0.0)
			{
				Particle particleI = m_ParticleSystem->getParticles()[m_ParticleSystem->Springs[m_ParticleSystem->NumSprings].Particle1];
				Particle particleJ = m_ParticleSystem->getParticles()[m_ParticleSystem->Springs[m_ParticleSystem->NumSprings].Particle2];
				Vector posI;
				particleI.getPos(posI);
				Vector posJ;
				particleJ.getPos(posJ);

				Vector sf;
				VecSubtract(sf, posI, posJ);
				double distBetween = VecLength(sf);
				m_ParticleSystem->Springs[m_ParticleSystem->NumSprings].RestLength = distBetween;
			}
			m_ParticleSystem->NumSprings++;
		}
	}
	else if (strcmp(argv[0], "fix") == 0)
	{
		if (argc != 2)
		{
			animTcl::OutputMessage("system %s: wrong number of params for set.", m_name.c_str());
			return TCL_ERROR;
		}
		else
		{
			m_Fixed[std::stoi(argv[1])] = true;
		}
	}
	else if (strcmp(argv[0], "integration") == 0)
	{
		if (argc != 3)
		{
			animTcl::OutputMessage("system %s: wrong number of params for set.", m_name.c_str());
			return TCL_ERROR;
		}
		else
		{
			if (strcmp(argv[1], "euler") == 0)
			{
				m_ActiveIntegrationMethod = euler;
			}
			else if (strcmp(argv[1], "symplectic") == 0)
			{
				m_ActiveIntegrationMethod = symplectic;
			}
			else if (strcmp(argv[1], "verlet") == 0)
			{
				m_ActiveIntegrationMethod = verlet;
			}

			m_TimeStep = std::stod(argv[2]);
		}
	}
	else if (strcmp(argv[0], "ground") == 0)
	{
		if (argc != 3)
		{
			animTcl::OutputMessage("system %s: wrong number of params for set.", m_name.c_str());
			return TCL_ERROR;
		}
		else
		{
		}
	}
	else if (strcmp(argv[0], "gravity") == 0)
	{
		if (argc != 2)
		{
			animTcl::OutputMessage("system %s: wrong number of params for set.", m_name.c_str());
			return TCL_ERROR;
		}
		else
		{
			m_Gravity = std::stod(argv[1]);
		}
	}
	else if (strcmp(argv[0], "drag") == 0)
	{
		if (argc != 2)
		{
			animTcl::OutputMessage("system %s: wrong number of params for set.", m_name.c_str());
			return TCL_ERROR;
		}
		else
		{
			m_DragCoef = -std::stod(argv[1]);
		}
	}
	glutPostRedisplay();
	return TCL_OK;
}