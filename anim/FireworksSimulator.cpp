#include "FireworksSimulator.h"

FireworksSimulator::FireworksSimulator(const std::string& name) :
	BaseSimulator(name)
{
}

FireworksSimulator::~FireworksSimulator()
{
}

int FireworksSimulator::step(double time)
{

	if ((time - mLastShotTime) > kFireworkInterval)
	{
		ShootFireWork();
		mLastShotTime = time;
		return 1;
	}
	return 0;
}

int FireworksSimulator::command(int argc, myCONST_SPEC char** argv)
{
	return 0;
}

void FireworksSimulator::ShootFireWork()
{

	bool success;
	// register a system
	ParticleSystem* particleSys = new ParticleSystem("partSys" + std::to_string(mNumFireworksShot));
	success = GlobalResourceManager::use()->addSystem(particleSys, true);

	// create random particle Sys
	double xPos = (2.0 * kExplosionCenterXRange + (double)rand()) / ((double)RAND_MAX / (2.0 * kExplosionCenterXRange)) - kExplosionCenterXRange;
	double yPos = (2.0 * kExplosionCenterYRange + (double)rand()) / ((double)RAND_MAX / (2.0 * kExplosionCenterYRange)) - kExplosionCenterYRange;
	double zPos = (2.0 * kExplosionCenterZRange + (double)rand()) / ((double)RAND_MAX / (2.0 * kExplosionCenterZRange)) - kExplosionCenterZRange;

	double vx = (2.0 * kExplosionInitVXRange + (double)rand()) / ((double)RAND_MAX / (2.0 * kExplosionInitVXRange)) - kExplosionInitVXRange;
	double vy = (2.0 * kExplosionInitVYRange + (double)rand()) / ((double)RAND_MAX / (2.0 * kExplosionInitVYRange)) - kExplosionInitVYRange;
	double vz = (2.0 * kExplosionInitVZRange + (double)rand()) / ((double)RAND_MAX / (2.0 * kExplosionInitVZRange)) - kExplosionInitVZRange;

	double R =(double)rand() / (double)RAND_MAX;
	double G = (double)rand() / (double)RAND_MAX;
	double B = (double)rand() / (double)RAND_MAX;

	double avgParticleLifetime = kLifeTimeAverage + ((2.0 * kLifeTimeMinMaxRange + (double)rand()) / ((double)RAND_MAX / (2.0 * kLifeTimeMinMaxRange)) - kLifeTimeMinMaxRange);

	particleSys->initParticleSystem(1000.0, xPos, yPos, zPos, vx, vy, vz, R, G, B, avgParticleLifetime);

	// register a system
	ParticlePhysicsSimulator* particlePhys = new ParticlePhysicsSimulator("partSim" + std::to_string(mNumFireworksShot));
	success = GlobalResourceManager::use()->addSimulator(particlePhys, true);

	particlePhys->Linksystem(particleSys->retrieveName(), 0);
	particlePhys->m_ActiveIntegrationMethod = ParticlePhysicsSimulator::IntegrationMethod::verlet;

	mNumFireworksShot++;
}
