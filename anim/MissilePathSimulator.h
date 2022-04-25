#ifndef _MissilePathSimulator_H_
#define _MissilePathSimulator_H_
#include "BaseSimulator.h"
#include "BaseSystem.h"
#include "HermiteSpline.h"
#include "Missile.h"
#include "Tank.h"
class MissilePathSimulator : public BaseSimulator
{
public:

	MissilePathSimulator(const std::string& name, Missile* missile, Tank* tank);

	int step(double time);
	int init(double time)
	{
		m_MissileObject->getState(m_pos0);
		m_MissileObject->setInitPos(m_pos0);
		m_MissileObject->setInitSpeed();
		return 0;
	};

	int command(int argc, myCONST_SPEC char** argv);

protected:

	Vector m_pos0; // initial position
	Vector m_pos;
	double m_vel;

	double m_prevTime;

	double m_currDist;

	bool m_finished;
	Missile* m_MissileObject;
	Tank* m_TankObject;
	HermiteSpline* m_Path;
};
#endif
