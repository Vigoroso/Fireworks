#pragma once
#include <GLModel/GLModel.h>
#include <shared/defs.h>
#include <util/util.h>
#include "animTcl.h"
#include "BaseSimulator.h"
#include "BaseSystem.h"
#include "ParticleSystem.h"
#include "Spring.h"

#include <string>

// a sample simulator

class ParticlePhysicsSimulator : public BaseSimulator
{
public:

	enum IntegrationMethod { euler, symplectic, verlet };
	ParticlePhysicsSimulator(const std::string& name);
	~ParticlePhysicsSimulator();

	int step(double time);
	int init(double time)
	{
		return 0;
	};

	int command(int argc, myCONST_SPEC char** argv);

	void SetParticleState(int particleIndex, double time);

	void Linksystem(std::string name, int numSprings);

	IntegrationMethod m_ActiveIntegrationMethod = euler;

	ParticleSystem* m_ParticleSystem;

protected:

	bool mFirstStep = true;
	bool* m_Fixed;
	double m_Gravity = -9.8;
	double m_DragCoef = -0.02;
	double m_TimeStep = 0.001;
	double m_BaseTempReduc = 10.0;
	double m_RandForceRange = 0.5;

	void AddSpringForces(Spring spring, Particle particleI, Particle particleJ, double* totalForce);

};