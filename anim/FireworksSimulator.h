#pragma once
#include <GLModel/GLModel.h>
#include <shared/defs.h>
#include <util/util.h>
#include "animTcl.h"
#include "BaseSimulator.h"
#include "BaseSystem.h"
#include "ParticleSystem.h"
#include "Spring.h"
#include "GlobalResourceManager.h"
#include "ParticlePhysicsSimulator.h"

#include <string>

// a sample simulator

class FireworksSimulator : public BaseSimulator
{
public:

	FireworksSimulator(const std::string& name);
	~FireworksSimulator();

	int step(double time);
	int init(double time)
	{
		return 0;
	};

	int command(int argc, myCONST_SPEC char** argv);

protected:

	double const kExplosionCenterXRange = 3.0;
	double const kExplosionCenterYRange = 3.0;
	double const kExplosionCenterZRange = 3.0;

	double const kExplosionInitVXRange = 3.0;
	double const kExplosionInitVYRange = 3.0;
	double const kExplosionInitVZRange = 3.0;

	double const kLifeTimeMinMaxRange = 2.0;
	double const kLifeTimeAverage = 5.0;

	double const kFireworkInterval = 1.0;
	double mLastShotTime = 0.0;
	int mNumFireworksShot = 0;


	void ShootFireWork();

};
